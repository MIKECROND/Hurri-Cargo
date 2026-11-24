/**
 * @file zdt_v5_port.c
 * @brief 张大头V5步进电机移植接口
 */

#include "zdt_v5_port.h"
#include "zdt_v5_drv.h"
#include "zdt_v5_engine.h"
#include "shell.h"
#include "log.h"
#include "shell_cmd_group.h"
#include "cmsis_os.h"
#include "usart.h"
#include "Events.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern osMutexId_t Motor_MutexHandle;
extern osMessageQueueId_t Usart6_Rx_DataHandle;
extern osMessageQueueId_t MotorCmdsHandle;
MotorStatus_t *g_motor_status[ZDT_STEP_NUM];
static uint32_t motor_last_rx_tick[ZDT_STEP_NUM]; // 各电机最近一次应答的 tick(0=从未应答)
static bool is_init = false; // 电机模块是否初始化
uint32_t step_update_time = 50; // 电机状态更新时间间隔（毫秒）
uint8_t rx6Buffer[USART6_RX_BUF_SIZE];

/**
 * @brief 运动 setpoint 邮箱：单槽"最新值覆盖"
 * @note  导航任务每 5ms 投递一次，控制任务取走上一次的值；未取走就被覆盖的旧值直接丢弃
 *        （丢弃数可通过 `step bus` 查看），保证电机永远只执行"最新"的速度指令。
 */
typedef struct {
    MotorCmd_t cmd[MOTOR_MOTION_BATCH_MAX]; // 本批运动命令
    uint8_t count; // 命令条数
    bool sync_after; // 本批发完后是否补一帧 0xFF66 同步触发
    bool pending; // 是否有待发 setpoint
    uint32_t post_tick; // 投递时刻(ms)，用于统计排队时延
} MotorMotionSlot_t;

static MotorMotionSlot_t g_motion_slot; // 运动 setpoint 邮箱（高优先级）
#if !MOTOR_CMD_LEGACY_CHAIN
static MotorCmd_t g_urgent_cmd[MOTOR_URGENT_QUEUE_LEN]; // 紧急通道环形缓冲（停止/使能）
static uint32_t g_urgent_tick[MOTOR_URGENT_QUEUE_LEN]; // 各紧急命令的投递时刻(ms)
static uint8_t g_urgent_head; // 紧急环读下标
static uint8_t g_urgent_tail; // 紧急环写下标
static uint8_t g_urgent_count; // 紧急环内条数
#endif
static MotorCmd_t g_sync_cmd; // 预置的同步触发命令（只有控制任务会发送）
static uint8_t g_raw_req[16]; // shell 原始帧请求（`step tool cmd`），由控制任务串行发送
static uint8_t g_raw_req_len;
static bool g_raw_req_pending;
static MotorBusStats_t g_bus_stats; // 命令链统计（step bus 查看）

#if MOTOR_BUS_STATS
#define MOTOR_STATS_INC(field) (g_bus_stats.field++)
#define MOTOR_STATS_ADD(field, v) (g_bus_stats.field += (uint32_t)(v))
#define MOTOR_STATS_MAX(field, v)                                                                                      \
    do {                                                                                                               \
        uint32_t _stat_v = (uint32_t)(v);                                                                              \
        if (_stat_v > g_bus_stats.field) g_bus_stats.field = _stat_v;                                                  \
    } while (0)
#define MOTOR_STATS_SET(field, v) (g_bus_stats.field = (uint8_t)(v))
#define MOTOR_STATS_SET32(field, v) (g_bus_stats.field = (uint32_t)(v))
#define MOTOR_STATS_RATE_TICK() (Motor_Stats_Update_Rate())

/**
 * @brief 初始化微秒级计时（DWT 周期计数器）
 * @note  仅用于统计"一帧发出要多久"（0.03~0.6ms），ms 级 tick 分辨率不够，故用 DWT。
 *        32 位 @168MHz 约 25.6s 回绕，无符号差值在单帧/单次测量的量级内正确。
 */
static void Motor_Stats_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t Motor_Now_Us(void) {
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000U));
}

/* 近 1s 速率统计（运动 setpoint 发送率、总线帧率），用于现场判断限频/带宽是否正常 */
static uint32_t g_rate_tick; // 上一次速率采样时刻(ms)
static uint32_t g_rate_motion; // 上次采样时的 motion_sent
static uint32_t g_rate_frames; // 上次采样时的 tx_frames

static void Motor_Stats_Update_Rate(void) {
    uint32_t now = osKernelGetTickCount();
    if ((now - g_rate_tick) >= 1000U) {
        g_bus_stats.motion_rate_1s = g_bus_stats.motion_sent - g_rate_motion;
        g_bus_stats.tx_rate_1s = g_bus_stats.tx_frames - g_rate_frames;
        g_rate_motion = g_bus_stats.motion_sent;
        g_rate_frames = g_bus_stats.tx_frames;
        g_rate_tick = now;
    }
}
#else
#define MOTOR_STATS_INC(field) ((void)0)
#define MOTOR_STATS_ADD(field, v) ((void)0)
#define MOTOR_STATS_MAX(field, v) ((void)0)
#define MOTOR_STATS_SET(field, v) ((void)0)
#define MOTOR_STATS_SET32(field, v) ((void)0)
#define MOTOR_STATS_RATE_TICK() ((void)0)
#endif /* MOTOR_BUS_STATS */

/**
 * @brief 普通命令之间的帧间隔
 * @note  驱动层已等到 UART6 TX 完成（最后一帧比特移出）才返回，这里只补"驱动器解析上一帧 +
 *        返回应答"需要的最短间隔，防粘包。旧实现是无条件 osDelay(2)(≈2ms)，把队列服务率
 *        压到约 480 条/s 以下（正是队列积压的根因之一）。
 */
static void Motor_InterFrame_Gap(void) {
#if MOTOR_CMD_LEGACY_CHAIN
    osDelay(MOTOR_CMD_LEGACY_GAP_MS);
#else
#if MOTOR_CMD_GAP_MS > 0
    osDelay(MOTOR_CMD_GAP_MS);
#endif
#endif
}

/**
 * @brief 多机打包帧 → 同步触发帧之间的间隔
 * @note  同步触发必须等所有驱动器已缓存完各自的子命令，间隔不能为 0。
 */
static void Motor_Motion_Sync_Gap(void) {
#if MOTOR_CMD_LEGACY_CHAIN
    osDelay(MOTOR_CMD_LEGACY_GAP_MS);
#else
#if MOTOR_MOTION_SYNC_GAP_MS > 0
    osDelay(MOTOR_MOTION_SYNC_GAP_MS);
#endif
#endif
}

/**
 * @brief 发送一帧并等待 UART6 TX 完成（同时统计发送耗时）
 * @param buf 帧缓冲
 * @param len 帧长
 * @return true=发送完成; false=DMA 启动失败或等待超时（异常保护，避免控制任务永久阻塞）
 */
static bool Motor_Tx_Frame(uint8_t *buf, uint8_t len) {
#if MOTOR_BUS_STATS
    uint32_t t_start_us = Motor_Now_Us();
#endif
    osEventFlagsClear(System_StatusHandle, UART6_TX_IDLE);
    if (HAL_UART_Transmit_DMA(&huart6, buf, len) != HAL_OK) {
        logError("UART DMA transmit failed");
        return false;
    }
#if MOTOR_TX_TIMEOUT_MS > 0
    uint32_t flags = osEventFlagsWait(System_StatusHandle, UART6_TX_IDLE, osFlagsWaitAny, MOTOR_TX_TIMEOUT_MS);
    if ((flags & osFlagsError) != 0) {
        /* DMA 完成回调丢失（UART 错误/调试器暂停/中断被长时间屏蔽）：中止本次发送并返回，
           不能让电机控制任务永久阻塞（否则整条命令链停摆、心跳保护会把所有电机停掉）。 */
        HAL_UART_AbortTransmit(&huart6);
        osEventFlagsClear(System_StatusHandle, UART6_TX_IDLE);
        MOTOR_STATS_INC(tx_timeout);
        logError("UART6 TX idle timeout (%lu ms)", (unsigned long)MOTOR_TX_TIMEOUT_MS);
        return false;
    }
#else
    osEventFlagsWait(System_StatusHandle, UART6_TX_IDLE, osFlagsWaitAny, osWaitForever);
#endif

#if MOTOR_BUS_STATS
    {
        uint32_t t_us = Motor_Now_Us() - t_start_us;
        MOTOR_STATS_INC(tx_frames);
        g_bus_stats.tx_time_last_us = t_us;
        MOTOR_STATS_MAX(tx_time_max_us, t_us);
    }
#endif
    return true;
}

/**
 * @brief 投递一个运动 setpoint（"最新值覆盖"邮箱，非阻塞）
 */
bool Motor_Send_Motion_Setpoint(const MotorCmd_t *cmds, uint8_t count, bool sync_after) {
    if (cmds == NULL || count == 0 || count > MOTOR_MOTION_BATCH_MAX) return false;
    if (!is_init) return false;

#if MOTOR_CMD_LEGACY_CHAIN
    /* 旧链路复现（仅台架 A/B 对照）：运动命令直接塞进共享 FIFO 队列，队列满时阻塞等待 */
    bool ok = true;
    for (uint8_t i = 0; i < count; i++) {
        if (!Motor_Send_Cmd((MotorCmd_t *)&cmds[i])) ok = false;
    }
    if (ok && sync_after) ok = Motor_Send_Cmd(&g_sync_cmd);
    return ok;
#else
    /* 拷贝约 100 字节（多机帧）在临界区内完成 ≈1~2us，远小于一帧的 10.8us/字节，
       不会影响 UART 接收；换来的好处是导航任务在这里"永不阻塞、永不丢最新值"。 */
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < count; i++) {
        g_motion_slot.cmd[i] = cmds[i];
    }
    g_motion_slot.count = count;
    g_motion_slot.sync_after = sync_after;
    if (g_motion_slot.pending) MOTOR_STATS_INC(motion_dropped); // 上一件还没发出 ⇒ 旧值作废
    g_motion_slot.pending = true;
    g_motion_slot.post_tick = osKernelGetTickCount();
    MOTOR_STATS_INC(motion_posted);
    taskEXIT_CRITICAL();
    return true;
#endif /* MOTOR_CMD_LEGACY_CHAIN */
}

/**
 * @brief 作废尚未发出的运动 setpoint
 */
void Motor_Cancel_Motion_Setpoint(void) {
    taskENTER_CRITICAL();
    g_motion_slot.pending = false;
    taskEXIT_CRITICAL();
}

/**
 * @brief 投递紧急命令（停止/使能）：非阻塞，控制任务优先发送
 */
bool Motor_Send_Cmd_Urgent(const MotorCmd_t *cmd) {
    if (cmd == NULL || !is_init) return false;

#if MOTOR_CMD_LEGACY_CHAIN
    return Motor_Send_Cmd((MotorCmd_t *)cmd);
#else
    taskENTER_CRITICAL();
    if (g_urgent_count >= MOTOR_URGENT_QUEUE_LEN) {
        /* 环满：丢最旧的一条，保证最新命令（通常是停止）一定能进去。
           停止是幂等的，连续多条停止丢一条不影响结果。 */
        g_urgent_head = (uint8_t)((g_urgent_head + 1) % MOTOR_URGENT_QUEUE_LEN);
        g_urgent_count--;
        MOTOR_STATS_INC(urgent_dropped);
    }
    g_urgent_cmd[g_urgent_tail] = *cmd;
    g_urgent_tick[g_urgent_tail] = osKernelGetTickCount();
    g_urgent_tail = (uint8_t)((g_urgent_tail + 1) % MOTOR_URGENT_QUEUE_LEN);
    g_urgent_count++;
    MOTOR_STATS_INC(urgent_posted);
    taskEXIT_CRITICAL();
    return true;
#endif /* MOTOR_CMD_LEGACY_CHAIN */
}

/**
 * @brief 从紧急环取一条命令（控制任务专用）
 */
#if MOTOR_CMD_LEGACY_CHAIN
static bool Motor_Urgent_Pop(MotorCmd_t *cmd, uint32_t *post_tick) {
    /* 旧链路复现模式：紧急命令已直接塞进共享 FIFO 队列，没有独立紧急通道 */
    (void)cmd;
    (void)post_tick;
    return false;
}
#else
static bool Motor_Urgent_Pop(MotorCmd_t *cmd, uint32_t *post_tick) {
    bool got = false;
    taskENTER_CRITICAL();
    if (g_urgent_count > 0) {
        *cmd = g_urgent_cmd[g_urgent_head];
        *post_tick = g_urgent_tick[g_urgent_head];
        g_urgent_head = (uint8_t)((g_urgent_head + 1) % MOTOR_URGENT_QUEUE_LEN);
        g_urgent_count--;
        MOTOR_STATS_INC(urgent_sent);
        got = true;
    }
    taskEXIT_CRITICAL();
    return got;
}
#endif /* MOTOR_CMD_LEGACY_CHAIN */

/**
 * @brief 从运动 setpoint 邮箱取最新一批（控制任务专用）
 */
static bool Motor_Motion_Take(
    MotorCmd_t cmds[], uint8_t *count, bool *sync_after, uint32_t *post_tick) {
    bool got = false;
    taskENTER_CRITICAL();
    if (g_motion_slot.pending) {
        for (uint8_t i = 0; i < g_motion_slot.count; i++) {
            cmds[i] = g_motion_slot.cmd[i];
        }
        *count = g_motion_slot.count;
        *sync_after = g_motion_slot.sync_after;
        *post_tick = g_motion_slot.post_tick;
        g_motion_slot.pending = false;
        MOTOR_STATS_INC(motion_sent);
        got = true;
    }
    taskEXIT_CRITICAL();
    return got;
}

/**
 * @brief 取 shell 原始帧请求（`step tool cmd`，无正式命令类型，只能按原始帧发）
 */
static bool Motor_Raw_Take(uint8_t *buf, uint8_t *len) {
    bool got = false;
    taskENTER_CRITICAL();
    if (g_raw_req_pending) {
        *len = g_raw_req_len;
        for (uint8_t i = 0; i < g_raw_req_len; i++) {
            buf[i] = g_raw_req[i];
        }
        g_raw_req_pending = false;
        got = true;
    }
    taskEXIT_CRITICAL();
    return got;
}

/**
 * @brief 请求发送一帧原始数据（把 UART 写操作收敛到控制任务，避免两个任务同时写 USART6）
 * @return true=已交给控制任务; false=上一帧还没发完/长度非法
 */
static bool Motor_Raw_Request(const uint8_t *buf, uint8_t len) {
    if (buf == NULL || len == 0 || len > sizeof(g_raw_req)) return false;
    bool ok = false;
    taskENTER_CRITICAL();
    if (!g_raw_req_pending) {
        for (uint8_t i = 0; i < len; i++) {
            g_raw_req[i] = buf[i];
        }
        g_raw_req_len = len;
        g_raw_req_pending = true;
        ok = true;
    }
    taskEXIT_CRITICAL();
    return ok;
}

/**
 * @brief 当前普通队列占用条数
 */
uint8_t Motor_Get_Queue_Usage(void) {
    if (!is_init || MotorCmdsHandle == NULL) return 0;
    return (uint8_t)osMessageQueueGetCount(MotorCmdsHandle);
}

const MotorBusStats_t *Motor_Bus_Stats_Get(void) {
    return &g_bus_stats;
}

void Motor_Bus_Stats_Reset(void) {
    taskENTER_CRITICAL();
    {
        /* queue_depth 是配置信息（启动时从内核取），清零时保留 */
        uint16_t depth = g_bus_stats.queue_depth;
        memset(&g_bus_stats, 0, sizeof(g_bus_stats));
        g_bus_stats.queue_depth = depth;
    }
#if MOTOR_BUS_STATS
    /* 速率统计基线同步到当前计数值，避免清零后 1s 内算出负数（无符号下溢） */
    g_rate_tick = osKernelGetTickCount();
    g_rate_motion = g_bus_stats.motion_sent;
    g_rate_frames = g_bus_stats.tx_frames;
#endif
    taskEXIT_CRITICAL();
}

void Motor_UART_TxCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        osEventFlagsSet(System_StatusHandle, UART6_TX_IDLE);
    }
}

void Motor_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART6) {
        if (huart->RxEventType == HAL_UART_RXEVENT_IDLE) {
            extern osMessageQueueId_t Usart6_Rx_DataHandle;

            Usart6_RxBuf_t rxBuf;
            rxBuf.len = Size;
            memcpy(rxBuf.data, rx6Buffer, Size);

            osStatus_t state = osMessageQueuePut(Usart6_Rx_DataHandle, &rxBuf, NULL, 0);
            if (state != osOK) {
                logWarning("Failed to put USART6 data into queue, code: %d", state);
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx6Buffer, USART6_RX_BUF_SIZE);
    }
}

void Motor_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);

        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;

        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx6Buffer, USART6_RX_BUF_SIZE);
    }
}

/**
 * @brief 初始化电机模块
 */
bool Motor_Init(void) {
    extern uint8_t rx6Buffer[USART6_RX_BUF_SIZE];

    for (uint8_t i = 0; i < ZDT_STEP_NUM; i++) {
        g_motor_status[i] = pvPortMalloc(sizeof(MotorStatus_t));
        if (!g_motor_status[i]) {
            for (uint8_t j = 0; j < i; j++) {
                vPortFree(g_motor_status[j]);
            }
            return false;
        }
        memset(g_motor_status[i], 0, sizeof(MotorStatus_t));
        if (!ZDT_V5_Register_Motor(i + 1, g_motor_status[i])) {
            for (uint8_t j = 0; j < i; j++) {
                vPortFree(g_motor_status[j]);
            }
            return false;
        }
    }

    MX_USART6_UART_Init();

    /* 命令链统计/计时初始化(DWT 周期计数器) + 预置同步触发命令 */
#if MOTOR_BUS_STATS
    Motor_Stats_Init();
    g_rate_tick = osKernelGetTickCount();
    g_bus_stats.queue_depth = (uint16_t)osMessageQueueGetCapacity(MotorCmdsHandle);
#endif
    memset(&g_sync_cmd, 0, sizeof(g_sync_cmd));
    g_sync_cmd.op_type = OP_CONTROL;
    g_sync_cmd.motor_id = 0; // 广播地址: 触发总线上所有已缓存命令的电机
    g_sync_cmd.type.ctrl.type = CTRL_SYNC;

    /* 回调必须先于发送命令注册: zdt_v5_port_send() 发送后要等 UART6_TX_IDLE,
       该标志由 Motor_UARTE_TxCallback 置位, 若注册滞后会永久阻塞 */
    if (HAL_UART_RegisterCallback(&huart6, HAL_UART_TX_COMPLETE_CB_ID, Motor_UART_TxCallback) != HAL_OK) {
        Error_Handler();
    }

    /* 注册接收事件回调函数 */
    if (HAL_UART_RegisterRxEventCallback(&huart6, Motor_UART_RxCallback) != HAL_OK) {
        Error_Handler();
    }

    /* 注册错误回调函数 */
    if (HAL_UART_RegisterCallback(&huart6, HAL_UART_ERROR_CB_ID, Motor_UART_ErrorCallback) != HAL_OK) {
        Error_Handler();
    }

    for (uint8_t i = 0; i < ZDT_STEP_NUM; i++) {
#if USE_HEARTBEAT
        /* 心跳保护时间 = 状态轮询周期 + 余量(余量太小会在系统繁忙时把电机误停) */
        ZDT_V5_Modify_Heart_Protect(i + 1, false, step_update_time + MOTOR_HEARTBEAT_MARGIN_MS);
#else
#if MOTOR_BROADCAST_READ_ID
        ZDT_V5_Read_Device_Info_Params(0, I_ID);
#else
        is_init = true;
        logWarning("MOTOR_BROADCAST_READ_ID is not enabled, the motor state cannot be detected");
#endif
#endif /* USE_HEARTBEAT */
        if (HAL_UART_Receive(&huart6, rx6Buffer, 4, 10) == HAL_OK) {
            is_init |= true;
        }
    }

    if (is_init) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx6Buffer, USART6_RX_BUF_SIZE);
    }

    return is_init;
}

/**
 * @brief 串口发送函数（必须）
 * @param cmd 命令指针
 * @param len 命令长度
 * @note 运行时除 Motor_Ctrl_Task 外不应有其它任务直接调用（USART6 是单写者资源）；
 *       shell 的原始帧调试命令走 Motor_Raw_Request() 交给控制任务串行发送。
 *       等待 TX 完成带超时保护，DMA 中断丢失时不会把整条命令链卡死。
 */
void zdt_v5_port_send(uint8_t *cmd, uint8_t len) {
    (void)Motor_Tx_Frame(cmd, len);
}

/**
 * @brief 发送电机控制命令到队列（普通通道: 状态轮询/机械臂/工具命令）
 * @param cmd 电机控制命令指针
 * @return true 成功发送，false 失败
 * @note 运动命令不再走这里（见 Motor_Send_Motion_Setpoint），所以 5ms 控制任务不会被本函数的
 *       100ms 阻塞拖住；这里的等待时间仍保留 100ms，保证机械臂/工具命令不会因偶发拥塞丢命令。
 */
bool Motor_Send_Cmd(MotorCmd_t *cmd) {
    if (!is_init) return false;
    osStatus_t status = osMessageQueuePut(MotorCmdsHandle, cmd, 0, 100);
#if MOTOR_BUS_STATS
    if (status == osOK) {
        uint32_t usage = osMessageQueueGetCount(MotorCmdsHandle);
        MOTOR_STATS_INC(queue_put_count);
        MOTOR_STATS_SET(queue_usage, usage);
        if (usage > g_bus_stats.queue_peak) g_bus_stats.queue_peak = (uint8_t)usage;
    } else {
        MOTOR_STATS_INC(queue_put_timeout);
    }
#endif
    return (status == osOK);
}

/**
 * @brief 查询电机距上次应答已经过的毫秒数
 * @param motor_id 电机总线 ID
 * @return 毫秒数; 从未收到应答或 ID 非法时返回 UINT32_MAX
 * @note 状态结构体是上电清零的, 电机没接/掉线时里面读到的是初始值而不是真实位置,
 *       用本函数可以区分"没数据"与"数据正好是 0"
 */
uint32_t Motor_Rx_Age_Ms(uint8_t motor_id) {
    if (motor_id < 1 || motor_id > ZDT_STEP_NUM) return UINT32_MAX;

    uint32_t tick = motor_last_rx_tick[motor_id - 1];
    if (tick == 0) return UINT32_MAX; // 从未收到过应答
    return osKernelGetTickCount() - tick;
}

/**
 * @brief 处理一条普通命令（统计状态轮询帧数）
 */
static void Motor_Handle_Normal_Cmd(MotorCmd_t *cmd) {
    if (cmd->op_type == OP_PARAM_READ) MOTOR_STATS_INC(poll_frames);
    ZDT_V5_Process_Cmd(cmd);
}

/**
 * @brief 电机控制任务（总线唯一写者）
 * @note 命令通道优先级（每发一帧前都会重新检查一遍，高优先级通道不会被积压堵住）：
 *       1) 紧急通道：停止/使能，实车安全相关，等待上限 = 一帧发送 + 一个间隙（<2ms）；
 *       2) 运动 setpoint 邮箱：只发最新值，过期 setpoint 直接丢弃，`step bus` 可见丢弃数；
 *       3) shell 原始帧调试命令；
 *       4) 普通 FIFO 队列：状态轮询/机械臂/工具命令（语义与旧实现一致）。
 *       空闲时以 MOTOR_CMD_IDLE_POLL_MS(默认 1ms) 为超时等普通队列：普通队列有数据立即唤醒，
 *       高优先级通道最坏唤醒延迟 ≤ 1ms。旧实现在这里每条命令后 osDelay(2) 且不区分优先级，
 *       队列服务率约 480 条/s < 需求约 680 条/s ⇒ 稳态排队 130ms 级，是本次修复的根因。
 */
void Motor_Ctrl_Task(void *argument) {
    (void)argument;
    MotorCmd_t cmd;
    MotorCmd_t motion_cmds[MOTOR_MOTION_BATCH_MAX];
    uint8_t raw_buf[16];
    uint8_t count = 0;
    uint8_t raw_len = 0;
    uint32_t post_tick = 0;
    bool sync_after = false;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);
    if (!is_init) {
        osMessageQueueDelete(MotorCmdsHandle);
        osThreadExit();
    }

    for (;;) {
        MOTOR_STATS_RATE_TICK(); // 每秒更新一次"setpoint 发送率/总线帧率"（不做其它工作）

        /* ---------- 1) 紧急通道：停止/使能，先于一切 ---------- */
        if (Motor_Urgent_Pop(&cmd, &post_tick)) {
            MOTOR_STATS_MAX(urgent_latency_max_ms, osKernelGetTickCount() - post_tick);
            ZDT_V5_Process_Cmd(&cmd);
            Motor_InterFrame_Gap();
            continue;
        }

        /* ---------- 2) 运动 setpoint 邮箱：只发最新值 ---------- */
        if (Motor_Motion_Take(motion_cmds, &count, &sync_after, &post_tick)) {
            uint32_t wait_ms = osKernelGetTickCount() - post_tick;
            MOTOR_STATS_ADD(motion_wait_sum_ms, wait_ms);
            MOTOR_STATS_MAX(motion_wait_max_ms, wait_ms);
            MOTOR_STATS_SET32(motion_wait_last_ms, wait_ms);
            if (wait_ms > MOTOR_MOTION_WAIT_SLOW_MS) MOTOR_STATS_INC(motion_wait_slow);
            for (uint8_t i = 0; i < count; i++) {
                if (i > 0) Motor_InterFrame_Gap();
                ZDT_V5_Process_Cmd(&motion_cmds[i]);
            }
#if MOTOR_SYNC_TRIGGER
            if (sync_after) {
                /* 各轮命令是"缓存等待同步"下发的, 必须补一帧 0xFF66 才会真正启动 */
                Motor_Motion_Sync_Gap();
                ZDT_V5_Process_Cmd(&g_sync_cmd);
            }
#else
            (void)sync_after;
#endif
            Motor_InterFrame_Gap();
            continue;
        }

        /* ---------- 3) shell 原始帧调试命令（把 UART 写操作收敛到本任务） ---------- */
        if (Motor_Raw_Take(raw_buf, &raw_len)) {
            (void)Motor_Tx_Frame(raw_buf, raw_len);
            Motor_InterFrame_Gap();
            continue;
        }

        /* ---------- 4) 普通 FIFO 队列（非阻塞取） ---------- */
        if (osMessageQueueGet(MotorCmdsHandle, &cmd, NULL, 0) == osOK) {
            Motor_Handle_Normal_Cmd(&cmd);
            Motor_InterFrame_Gap();
            continue;
        }

        /* ---------- 5) 空闲：等普通队列，同时用超时给高优先级通道兜底 ---------- */
        if (osMessageQueueGet(MotorCmdsHandle, &cmd, NULL, MOTOR_CMD_IDLE_POLL_MS) == osOK) {
            Motor_Handle_Normal_Cmd(&cmd);
            Motor_InterFrame_Gap();
        }
    }
}

/**
 * @brief 电机信息处理任务
 */
void Motor_Receive_Task(void *argument) {
    (void)argument;
    Usart6_RxBuf_t rxBuf;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);
    if (!is_init) {
        osMessageQueueDelete(Usart6_Rx_DataHandle);
        osThreadExit();
    }

    for (;;) {
        if (osMessageQueueGet(Usart6_Rx_DataHandle, &rxBuf, NULL, osWaitForever) == osOK) {
            // 报文首字节是电机地址, 记录该电机的应答时间(用于判断其是否在线)
            if (rxBuf.data[0] >= 1 && rxBuf.data[0] <= ZDT_STEP_NUM) {
                motor_last_rx_tick[rxBuf.data[0] - 1] = osKernelGetTickCount();
            }
            if (osMutexAcquire(Motor_MutexHandle, osWaitForever) == osOK) {
                ZDT_V5_Receive(rxBuf.data, rxBuf.len);
                osMutexRelease(Motor_MutexHandle);
            }
        }
    }
}

/**
 * @brief 电机状态更新任务
 */
void Motor_Update_Task(void *argument) {
    (void)argument;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);
    if (!is_init) osThreadExit();

    extern void Motor_Update(MotorCmd_t * cmd, uint8_t motor_id);
    MotorCmd_t cmd;

    for (;;) {
        osDelay(step_update_time);
        for (uint8_t i = 0; i < ZDT_STEP_NUM; i++) {
            if (g_motor_status[i] != NULL) {
                Motor_Update(&cmd, g_motor_status[i]->motor_id);
            }
        }
    }
}

#if MOTOR_TRIGGER_RESET_POS
/**
 * @brief 电机位置清零
 */
void Motor_zero(uint8_t motor_id) {
    MotorCmd_t cmd;
    cmd.op_type = OP_TRIGGER;
    cmd.motor_id = motor_id;

    MotorTrigger_t trigger;
    trigger.type = TRIG_RESET_POS;
    cmd.type.trigger = trigger;
    Motor_Send_Cmd(&cmd);
}
#endif /* MOTOR_TRIGGER_RESET_POS */

#if MOTOR_BROADCAST_READ_ID
/**
 * @brief 检查电机是否在线
 * @param motor_id 电机ID
 * @return true 电机在线，false 电机不在线
 */
static bool Motor_isonline(uint8_t motor_id) {
    MotorStatus_t *motor = g_motor_status[motor_id - 1];
    motor->is_online = false;
    ZDT_V5_Read_Device_Info_Params(0, I_ID);
    osDelay(step_update_time + 50);
    return motor->is_online;
}
#endif /* MOTOR_BROADCAST_READ_ID */

#if MOTOR_CMD_ENABLE
/**
 * @brief 投递"广播使能/失能"命令（紧急通道: 先于运动 setpoint 与状态轮询发送）
 * @param enable true=使能(锁轴), false=失能(松轴, 急停)
 * @return true=已投递; false=驱动未初始化(此时直接发也没有意义, 仍保留直接下发兜底)
 */
static bool Motor_Send_Broadcast_Enable(bool enable) {
    MotorCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.op_type = OP_CONTROL;
    cmd.motor_id = 0;
    cmd.type.ctrl.type = CTRL_ENABLE;
    cmd.type.ctrl.p.en.enable = enable;
    cmd.type.ctrl.p.en.sync = false;
    return Motor_Send_Cmd_Urgent(&cmd);
}

/**
 * @brief 紧急停止
 * @note 走紧急通道：不排在状态轮询/运动 setpoint 后面（旧实现直接从 shell 任务写 USART6，
 *       会与控制任务的发送争抢 DMA；现在统一由控制任务串行发送，且优先级最高）。
 */
static void Step_ES(int argc, char *argv[]) {
    if (argc == 2 && strcmp(argv[1], "r") == 0) {
        if (!Motor_Send_Broadcast_Enable(true)) {
            ZDT_V5_En_Control(0, true, false);
        }
        logPrintln("Motor Enabled");
    } else {
        if (!Motor_Send_Broadcast_Enable(false)) {
            ZDT_V5_En_Control(0, false, false);
        }
        logPrintln("Motor Emergency Stop\r\nUse: es r to reset");
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 es,
                 Step_ES,
                 Emergency Stop);

/**
 * @brief 紧急停止按键导出
 */
static void ES_Key(void) {
    Step_ES(0, NULL);
}
SHELL_EXPORT_KEY(SHELL_CMD_PERMISSION(0), 0x18000000, ES_Key, ^X);
#endif /* MOTOR_CMD_ENABLE */

#if MOTOR_VELOCITY_MODE
/**
 * @brief 单个电机速度控制
 */
static void Motor_vel_Shell(int argc, char *argv[]) {
    if (!is_init) {
        logWarning("Motor module not initialized");
        return;
    }

    if (argc != 5) {
        logPrintln("Usage: vel [id] [dir] [vel] [acc]");
        return;
    }

    uint8_t motor_id = atoi(argv[1]);
    uint8_t dir = atoi(argv[2]);
    uint16_t vel = atoi(argv[3]);
    uint16_t acc = atoi(argv[4]);

    MotorCmd_t cmd;
    cmd.op_type = OP_CONTROL;
    cmd.motor_id = motor_id;

    MotorCtrl_t ctrl;
    ctrl.type = CTRL_VEL;
    ctrl.p.vel.dir = dir;
    ctrl.p.vel.vel = vel;
    ctrl.p.vel.acc = acc;
    ctrl.p.vel.sync = false;
    cmd.type.ctrl = ctrl;

    if (Motor_Send_Cmd(&cmd)) {
        logPrintln("Velocity cmd sent to motor %d: dir=%d, vel=%d, acc=%d", motor_id, dir, vel, acc);
    } else {
        logPrintln("Failed to send velocity command");
    }
}
#endif /* MOTOR_VELOCITY_MODE */

static void Tool_Help(void) {
    logPrintln("Usage: tool COMMAND [value...]\r\n"
               "\r\n"
               "commands:\r\n"
               "  cmd       Send Motor Command\r\n"
#if MOTOR_BROADCAST_READ_ID
               "  online    Check Motor Online\r\n"
#endif
#if MOTOR_CMD_ENABLE
               "  en        Enable/Disable Motor\r\n"
#endif
#if MOTOR_VELOCITY_MODE
               "  found     Find Motor\r\n"
#endif
#if MOTOR_CMD_STOP
               "  stop      Stop Motor\r\n"
#endif
#if MOTOR_TRIGGER_RESET_POS
               "  zero      Reset Motor Position to Zero\r\n"
#endif
#if MOTOR_HOME_TRIGGER
               "  home      Trigger Motor Homing\r\n"
#endif
#if MOTOR_DRIVER_POS_WINDOW
               "  window    Set Motor Window\r\n"
#endif
#if MOTOR_PID_WRITE
               "  pid       Set motor PID\r\n"
#endif
#if MOTOR_INTEGRAL_LIMIT_WRITE
               "  ilimit    Set motor integral limit\r\n"
#endif
#if MOTOR_TRIGGER_ENCODER_CALIB
               "  cal       Calibrate Motor Encoder\r\n"
#endif
#if MOTOR_TRIGGER_RESTART
               "  rstrt     Restart Motor\r\n"
#endif
               "  time      View or Set Update Time\r\n");
}

#if MOTOR_POS_MODE
/**
 * @brief 单个电机位置控制
 */
static void Motor_pos_Shell(int argc, char *argv[]) {
    if (!is_init) {
        logWarning("Motor module not initialized");
        return;
    }

#if CURRENT_FIRMWARE == FIRMWARE_X
    if (argc != 8) {
        logPrintln("Usage: pos [id] [dir] [vel] [acc] [angle] [mode] [dec]\r\n"
                   "mode: 0-relative to last target, 1-absolute, 2-relative to current");
        return;
    }
#elif CURRENT_FIRMWARE == FIRMWARE_EMM
    if (argc != 7) {
        logPrintln("Usage: pos [id] [dir] [vel] [acc] [target] [mode]\r\n"
                   "mode: 0-relative to last target, 1-absolute, 2-relative to current");
        return;
    }
#endif

    uint8_t motor_id = atoi(argv[1]);
    uint8_t dir = atoi(argv[2]);
    uint16_t vel = atoi(argv[3]);
    uint16_t acc = atoi(argv[4]);
    int32_t target_angle = atoi(argv[5]);
    uint8_t mode = atoi(argv[6]);

    MotorCmd_t cmd;
    cmd.op_type = OP_CONTROL;
    cmd.motor_id = motor_id;

    MotorCtrl_t ctrl;
    ctrl.type = CTRL_POS;
    ctrl.p.pos.dir = dir;
    ctrl.p.pos.vel = vel;
    ctrl.p.pos.acc = acc;
#if CURRENT_FIRMWARE == FIRMWARE_X
    uint16_t dec = atoi(argv[6]);
    ctrl.p.pos.dec = dec;
#endif
    ctrl.p.pos.target = target_angle;
    ctrl.p.pos.mode = mode;
    ctrl.p.pos.sync = false;
    cmd.type.ctrl = ctrl;

    if (Motor_Send_Cmd(&cmd)) {
#if CURRENT_FIRMWARE == FIRMWARE_X
        logPrintln("Position cmd sent to motor %d: dir=%d, vel=%d, acc=%d, dec= %d, target/angle=%ld, mode=%d",
                   motor_id,
                   dir,
                   vel,
                   acc,
                   dec,
                   target_angle,
                   mode);
#elif CURRENT_FIRMWARE == FIRMWARE_EMM
        logPrintln("Position cmd sent to motor %d: dir=%d, vel=%d, acc=%d, target/angle=%ld, mode=%d",
                   motor_id,
                   dir,
                   vel,
                   acc,
                   target_angle,
                   mode);
#endif
    } else {
        logPrintln("Failed to send position command");
    }
}
#endif /* MOTOR_POS_MODE */

/**
 * @brief 电机实用工具组
 */
static void Motor_tool_Shell(int argc, char *argv[]) {
    if (!is_init) {
        logWarning("Motor module not initialized");
        return;
    }

    if (argc < 2) {
        Tool_Help();
        return;
    }
    MotorCmd_t cmd;
    MotorCtrl_t ctrl;
    MotorTrigger_t trigger;
    MotorParamWrite_t write;
    uint8_t motor_id;
#if MOTOR_POS_WINDOW_WRITE || MOTOR_PID_WRITE || MOTOR_INTEGRAL_LIMIT_WRITE
    bool save = false;
#endif
    char *endptr;

    if (strcmp(argv[1], "cmd") == 0) {
        if (argc != 3) {
            logPrintln("Usage: tool cmd [cmd]");
            return;
        }

        char *cmd_str = argv[2];
        uint8_t cmd_buffer[16];
        uint8_t cmd_len = 0;

        char *token = strtok(cmd_str, " ");
        while (token != NULL && cmd_len < sizeof(cmd_buffer)) {
            uint8_t byte = 0;
            sscanf(token, "%2hhx", &byte);
            cmd_buffer[cmd_len++] = byte;
            token = strtok(NULL, " ");
        }
        /* 交给 Motor_Ctrl_Task 串行发送: 若直接从 shell 任务写 USART6, 会与正在发送的
           控制任务抢同一个 UART(DMA HAL_BUSY), 导致两帧都丢。 */
        if (!Motor_Raw_Request(cmd_buffer, cmd_len)) {
            logPrintln("Failed: motor control task busy, retry");
        }
    }
#if MOTOR_CMD_ENABLE
    else if (strcmp(argv[1], "en") == 0) {
        if (argc != 4) {
            logPrintln("Usage: tool en [id] [state]");
            return;
        }
        long val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        motor_id = (uint8_t)val;
        cmd.op_type = OP_CONTROL;
        cmd.motor_id = motor_id;

        bool state = atoi(argv[3]);
        ctrl.type = CTRL_ENABLE;
        ctrl.p.en.enable = state;
        ctrl.p.en.sync = false;
        cmd.type.ctrl = ctrl;
        if (Motor_Send_Cmd(&cmd))
            logPrintln("Motor %d is %s", motor_id, state ? "enabled" : "disabled");
        else
            logPrintln("Failed to sen command");
    }
#endif /* MOTOR_CMD_ENABLE */
#if MOTOR_CMD_STOP
    else if (strcmp(argv[1], "stop") == 0) {
        if (argc != 3) {
            logPrintln("Usage: tool stop [id]");
            return;
        }
        long val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        motor_id = (uint8_t)val;
        cmd.op_type = OP_CONTROL;
        cmd.motor_id = motor_id;

        ctrl.type = CTRL_STOP;
        ctrl.p.stop.sync = false;
        cmd.type.ctrl = ctrl;
        if (Motor_Send_Cmd(&cmd))
            logPrintln("Motor %d is stopped", motor_id);
        else
            logPrintln("Failed to sen command");
    }
#endif /* MOTOR_CMD_STOP */
#if MOTOR_TRIGGER_RESET_POS
    else if (strcmp(argv[1], "zero") == 0) {
        if (argc != 3) {
            logPrintln("Usage: tool zero [id]");
            return;
        }
        long val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        motor_id = (uint8_t)val;
        cmd.op_type = OP_TRIGGER;
        cmd.motor_id = motor_id;

        trigger.type = TRIG_RESET_POS;
        cmd.type.trigger = trigger;
        if (Motor_Send_Cmd(&cmd))
            logPrintln("Motor %d position reset to zero", motor_id);
        else
            logPrintln("Failed to sen command");
    }
#endif /* MOTOR_TRIGGER_RESET_POS */
#if MOTOR_HOME_TRIGGER
    else if (strcmp(argv[1], "home") == 0) {
        if (argc != 3) {
            logPrintln("Usage: tool home [id]");
            return;
        }
        long val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        motor_id = (uint8_t)val;
        cmd.op_type = OP_TRIGGER;
        cmd.motor_id = motor_id;

        trigger.type = TRIG_HOME_RETURN;
        trigger.p.home.mode = 0;
        trigger.p.home.sync = false;
        cmd.type.trigger = trigger;
        if (Motor_Send_Cmd(&cmd))
            logPrintln("Motor %d homing triggered", motor_id);
        else
            logPrintln("Failed to sen command");
    }
#endif /* MOTOR_HOME_TRIGGER */
#if MOTOR_TRIGGER_ENCODER_CALIB
    else if (strcmp(argv[1], "cal") == 0) {
        if (argc != 3) {
            logPrintln("Usage: tool cal [id]");
            return;
        }
        long val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        motor_id = (uint8_t)val;
        cmd.op_type = OP_TRIGGER;
        cmd.motor_id = motor_id;

        trigger.type = TRIG_ENCODER_CALIB;
        cmd.type.trigger = trigger;
        if (Motor_Send_Cmd(&cmd))
            logPrintln("Starting calibration for motor %d...", motor_id);
        else
            logPrintln("Failed to sen command");
    }
#endif /* MOTOR_TRIGGER_ENCODER_CALIB */
    else if (strcmp(argv[1], "time") == 0) {
        if (argc > 3) {
            logPrintln("Usage: tool time [time]");
            return;
        } else if (argc == 2) {
            logPrintln("current time: %d ms", step_update_time);
            return;
        }

#if MOTOR_HEARTBEAT_WRITE
        /* 修改状态轮询周期必须同步改写"所有电机"的心跳保护时间（原实现用了未初始化的
           motor_id → 只有一个随机电机被改写，其它电机仍按旧窗口保护，容易误停转）。
           新保护时间 = 新轮询周期 + 余量，改完立即生效并作用于每个总线电机。 */
        uint32_t new_period_ms = (uint32_t)atoi(argv[2]);
        for (uint8_t i = 1; i <= ZDT_STEP_NUM; i++) {
            cmd.motor_id = i;
            cmd.op_type = OP_PARAM_WRITE;

            write.type = PARAM_HEARTBEAT;
            write.p.heartbeat.save = false;
            write.p.heartbeat.time_ms = new_period_ms + MOTOR_HEARTBEAT_MARGIN_MS;
            cmd.type.write = write;
            if (!Motor_Send_Cmd(&cmd)) {
                logPrintln("Failed to send heartbeat command for motor %d", i);
                return;
            }
        }
        step_update_time = new_period_ms;
        logPrintln("Motor update time set to: %lu ms (heartbeat %lu ms for all %d motors)",
                   (unsigned long)step_update_time,
                   (unsigned long)(new_period_ms + MOTOR_HEARTBEAT_MARGIN_MS),
                   (int)ZDT_STEP_NUM);
#else
        step_update_time = atoi(argv[2]);
        logPrintln("Motor update time set to: %d ms", step_update_time);
#endif /* MOTOR_HEARTBEAT_WRITE */
    }
#if MOTOR_BROADCAST_READ_ID
    else if (strcmp(argv[1], "online") == 0) {
        if (argc != 3) {
            logPrintln("Usage: tool online [id]");
            return;
        }
        long val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        motor_id = (uint8_t)val;

        logPrintln("Motor %d is %s", motor_id, Motor_isonline(motor_id) ? "online" : "offline");
    }
#endif /* MOTOR_BROADCAST_READ_ID */
#if MOTOR_VELOCITY_MODE
    else if (strcmp(argv[1], "found") == 0) {
        if (argc != 3) {
            logPrintln("Usage: tool found [id]");
            return;
        }
        long val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        motor_id = (uint8_t)val;
        cmd.op_type = OP_CONTROL;
        cmd.motor_id = motor_id;

        ctrl.type = CTRL_VEL;
        ctrl.p.vel.dir = 0;
        ctrl.p.vel.vel = 300;
        ctrl.p.vel.acc = 100;
        ctrl.p.vel.sync = false;
        cmd.type.ctrl = ctrl;
        if (!Motor_Send_Cmd(&cmd)) logPrintln("Failed to send velocity command");
        logPrintln("Motor %d running 3s", motor_id);
        osDelay(3000);
        ctrl.p.vel.vel = 0;
        cmd.type.ctrl = ctrl;
        /* 停止命令有限重试（旧实现是 while(!Motor_Send_Cmd())，驱动异常时会死循环卡住 shell） */
        for (uint8_t attempt = 0; attempt < 5U; attempt++) {
            if (Motor_Send_Cmd(&cmd)) break;
            osDelay(10);
        }
    }
#endif /* MOTOR_VELOCITY_MODE */
#if MOTOR_POS_WINDOW_WRITE
    else if (strcmp(argv[1], "window") == 0) {
        if (argc != 5) {
            logPrintln("Usage: tool window [id] [deg*10] [save]");
            return;
        }
        long id_val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        long win_val = strtol(argv[3], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid window value: %s", argv[3]);
            return;
        }
        long save_val = strtol(argv[4], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid save value: %s", argv[4]);
            return;
        }
        motor_id = (uint8_t)id_val;
        uint16_t window = (uint16_t)win_val;
        save = (save_val != 0);

        cmd.op_type = OP_PARAM_WRITE;
        cmd.motor_id = motor_id;

        write.type = PARAM_POS_WINDOW;
        write.p.pos_window.window = window;
        write.p.pos_window.save = save;
        cmd.type.write = write;
        if (Motor_Send_Cmd(&cmd))
            logPrintln("Motor %d window set: %d save=%d", motor_id, window, save);
        else
            logPrintln("Failed to sen command");

    }
#endif /* MOTOR_POS_WINDOW_WRITE */
#if MOTOR_PID_WRITE
    else if (strcmp(argv[1], "pid") == 0) {
#if CURRENT_FIRMWARE == FIRMWARE_EMM
        /* Emm: tool pid [id] [kp] [ki] [kd] [save] → 6 args */
        int req_argc = 6;
#elif CURRENT_FIRMWARE == FIRMWARE_X
        /* X:   tool pid [id] [trap_kp] [direct_kp] [vel_kp] [vel_ki] [save] → 7 args */
        int req_argc = 7;
#endif
        if (argc == 3 || argc == req_argc) {
            long id_val = strtol(argv[2], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid id value: %s", argv[2]);
                return;
            }
            motor_id = (uint8_t)id_val;

            if (argc == 3) {
                if (motor_id < 1 || motor_id > ZDT_STEP_NUM) {
                    logPrintln("Motor must be 1-%d", ZDT_STEP_NUM);
                    return;
                }
                if (osMutexAcquire(Motor_MutexHandle, osWaitForever) == osOK) {
                    MotorStatus_t *m = &g_motor_status->motors[motor_id - 1];
#if CURRENT_FIRMWARE == FIRMWARE_EMM
                    logPrintln("id=%d kp=%lu ki=%lu kd=%lu",
                               motor_id,
                               (unsigned long)m->kp,
                               (unsigned long)m->ki,
                               (unsigned long)m->kd);
#elif CURRENT_FIRMWARE == FIRMWARE_X
                    logPrintln("id=%d trap_kp=%lu direct_kp=%lu vel_kp=%lu vel_ki=%lu",
                               motor_id,
                               (unsigned long)m->trapezoidal_kp,
                               (unsigned long)m->direct_kp,
                               (unsigned long)m->vel_kp,
                               (unsigned long)m->vel_ki);
#endif
                    osMutexRelease(Motor_MutexHandle);
                }
                return;
            }

            long save_val;

#if CURRENT_FIRMWARE == FIRMWARE_EMM
            uint32_t kp = (uint32_t)strtoul(argv[3], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid kp value: %s", argv[3]);
                return;
            }
            uint32_t ki = (uint32_t)strtoul(argv[4], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid ki value: %s", argv[4]);
                return;
            }
            uint32_t kd = (uint32_t)strtoul(argv[5], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid kd value: %s", argv[5]);
                return;
            }
            save_val = strtol(argv[6], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid save value: %s", argv[6]);
                return;
            }
            save = (save_val != 0);

            cmd.motor_id = motor_id;
            cmd.op_type = OP_PARAM_WRITE;

            write.type = PARAM_PID;
            write.p.pid.kp = kp;
            write.p.pid.ki = ki;
            write.p.pid.kd = kd;
            write.p.pid.save = save;
            cmd.type.write = write;
            if (Motor_Send_Cmd(&cmd))
                logPrintln("Motor %d PID set: kp=%lu ki=%lu kd=%lu save=%d",
                           motor_id,
                           (unsigned long)kp,
                           (unsigned long)ki,
                           (unsigned long)kd,
                           save);
            else
                logPrintln("Failed to sen command");

#elif CURRENT_FIRMWARE == FIRMWARE_X
            uint32_t trap_kp = (uint32_t)strtoul(argv[3], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid trapezoidal_kp value: %s", argv[3]);
                return;
            }
            uint32_t direct_kp = (uint32_t)strtoul(argv[4], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid direct_kp value: %s", argv[4]);
                return;
            }
            uint32_t vel_kp = (uint32_t)strtoul(argv[5], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid vel_kp value: %s", argv[5]);
                return;
            }
            uint32_t vel_ki_val = (uint32_t)strtoul(argv[6], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid vel_ki value: %s", argv[6]);
                return;
            }
            save_val = strtol(argv[7], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid save value: %s", argv[7]);
                return;
            }
            save = (save_val != 0);

            cmd.motor_id = motor_id;
            cmd.op_type = OP_PARAM_WRITE;

            write.type = PARAM_PID;
            write.p.pid.trapezoidal_kp = trap_kp;
            write.p.pid.direct_kp = direct_kp;
            write.p.pid.vel_kp = vel_kp;
            write.p.pid.vel_ki = vel_ki_val;
            write.p.pid.save = save;
            cmd.type.write = write;
            if (Motor_Send_Cmd(&cmd))
                logPrintln("Motor %d PID set: trap_kp=%lu direct_kp=%lu vel_kp=%lu vel_ki=%lu save=%d",
                           motor_id,
                           (unsigned long)trap_kp,
                           (unsigned long)direct_kp,
                           (unsigned long)vel_kp,
                           (unsigned long)vel_ki_val,
                           save);
            else
                logPrintln("Failed to sen command");
#endif
        } else {
            logPrintln("Usage: tool pid [id]                - show motor PID parameters");
#if CURRENT_FIRMWARE == FIRMWARE_EMM
            logPrintln("       tool pid [id] [kp] [ki] [kd] [save]");
#elif CURRENT_FIRMWARE == FIRMWARE_X
            logPrintln("       tool pid [id] [trap_kp] [direct_kp] [vel_kp] [vel_ki] [save]");
#endif
            return;
        }
    }
#endif /* MOTOR_PID_WRITE */
#if MOTOR_INTEGRAL_LIMIT_WRITE
    else if (strcmp(argv[1], "ilimit") == 0) {
        int req_argc = 5;
        if (argc == 3 || argc == req_argc) {
            long id_val = strtol(argv[2], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid id value: %s", argv[2]);
                return;
            }
            motor_id = (uint8_t)id_val;

            if (argc == 3) {
                if (motor_id < 1 || motor_id > ZDT_STEP_NUM) {
                    logPrintln("Motor must be 1-%d", ZDT_STEP_NUM);
                    return;
                }
                if (osMutexAcquire(Motor_MutexHandle, osWaitForever) == osOK) {
                    MotorStatus_t *m = &g_motor_status->motors[motor_id - 1];
                    logPrintln("id=%d integral_limit=%lu", motor_id, (unsigned long)m->integral_limit);
                    osMutexRelease(Motor_MutexHandle);
                }
                return;
            }

            uint32_t ilimit_val = (uint32_t)strtoul(argv[3], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid value: %s", argv[3]);
                return;
            }

            long save_val = strtol(argv[4], &endptr, 10);
            if (*endptr != '\0') {
                logPrintln("invalid save value: %s", argv[4]);
                return;
            }
            bool save = (save_val != 0);

            cmd.motor_id = motor_id;
            cmd.op_type = OP_PARAM_WRITE;

            write.type = PARAM_INTEGRAL_LIMIT;
            write.p.integral_limit.value = ilimit_val;
            write.p.integral_limit.save = save;
            cmd.type.write = write;
            if (Motor_Send_Cmd(&cmd))
                logPrintln("Motor %d integral_limit set: value=%lu save=%d", motor_id, (unsigned long)ilimit_val, save);
            else
                logPrintln("Failed to send command");
        } else {
            logPrintln("Usage: tool ilimit [id]                - show motor integral_limit");
            logPrintln("       tool ilimit [id] [value] [save] - set motor integral_limit");
            return;
        }
    }
#endif
#if MOTOR_TRIGGER_RESTART
    else if (strcmp(argv[1], "rstrt") == 0) {
        if (argc != 3) {
            logPrintln("Usage: tool rstrt [id]");
            return;
        }
        long val = strtol(argv[2], &endptr, 10);
        if (*endptr != '\0') {
            logPrintln("invalid id value: %s", argv[2]);
            return;
        }
        motor_id = (uint8_t)val;
        cmd.motor_id = motor_id;
        cmd.op_type = OP_TRIGGER;

        trigger.type = TRIG_RESTART;
        cmd.type.trigger = trigger;
        if (Motor_Send_Cmd(&cmd))
            logPrintln("Motor %d restart success", motor_id);
        else
            logPrintln("Failed to send command");
    }
#endif /* MOTOR_TRIGGER_RESTART */
    else {
        logPrintln("Invalid command: %s", argv[1]);
        Tool_Help();
    }
}

#if USE_VIEW
extern void Motor_View_Shell(void);
#endif

/**
 * @brief 打印电机命令链统计（`step bus` / `car bus` 共用，默认不打印任何周期日志）
 *
 * 判读方法（2026-11 修复目标）：
 *   - motion rate：稳态巡航应接近 50Hz（setpoint 去重生效），加减速段可到 200Hz；
 *   - motion drop%：被覆盖丢弃的比例，应接近 0%（>10% 说明发送能力不足，看 rate 与 gap 配置）；
 *   - motion wait max 应长期 <5ms；修复前等效排队 ≈ 队列深度(64) × 每条 2.1ms ≈ 134ms；
 *   - motion wait slow(>20ms) 应长期为 0；
 *   - urgent latency max 应 <2ms（停止/使能命令的等待上限）；
 *   - queue peak 正常应 ≤2（状态轮询每 50ms 一批，运动命令已不走队列）；
 *   - queue put_timeout 应长期为 0；
 *   - tx frames/s 可用于估算总线占用：多机帧 37B≈403us、同步帧 4B≈44us、轮询帧 3B≈33us。
 */
void Motor_Bus_Stats_Print(void) {
#if !MOTOR_BUS_STATS
    logPrintln("[bus] statistics disabled (MOTOR_BUS_STATS=0)");
#else
    const MotorBusStats_t *s = Motor_Bus_Stats_Get();
    uint32_t motion_avg = (s->motion_sent > 0) ? (s->motion_wait_sum_ms / s->motion_sent) : 0;
    /* 丢弃比例 = 丢弃 / 投递（万分比, 避免浮点打印） */
    uint32_t drop_permille = (s->motion_posted > 0) ? (uint32_t)(((uint64_t)s->motion_dropped * 1000U) / s->motion_posted) : 0;

    logPrintln("[bus] chain=%s gap=%dms sync_gap=%dms idle_poll=%dms tx_timeout=%dms queue_depth=%u",
#if MOTOR_CMD_LEGACY_CHAIN
               "legacy(FIFO+osDelay)",
#else
               "priority(mailbox+urgent)",
#endif
               (int)MOTOR_CMD_GAP_MS,
               (int)MOTOR_MOTION_SYNC_GAP_MS,
               (int)MOTOR_CMD_IDLE_POLL_MS,
               (int)MOTOR_TX_TIMEOUT_MS,
               (unsigned)s->queue_depth);
    logPrintln("[bus] motion: post=%lu sent=%lu drop=%lu(%lu.%lu%%) rate=%luHz wait(last/avg/max)=%lu/%lu/%lu ms slow=%lu",
               (unsigned long)s->motion_posted,
               (unsigned long)s->motion_sent,
               (unsigned long)s->motion_dropped,
               (unsigned long)(drop_permille / 10U),
               (unsigned long)(drop_permille % 10U),
               (unsigned long)s->motion_rate_1s,
               (unsigned long)s->motion_wait_last_ms,
               (unsigned long)motion_avg,
               (unsigned long)s->motion_wait_max_ms,
               (unsigned long)s->motion_wait_slow);
    logPrintln("[bus] urgent: post=%lu sent=%lu drop=%lu latency_max=%lu ms",
               (unsigned long)s->urgent_posted,
               (unsigned long)s->urgent_sent,
               (unsigned long)s->urgent_dropped,
               (unsigned long)s->urgent_latency_max_ms);
    logPrintln("[bus] queue : usage=%u peak=%u put=%lu timeout=%lu",
               (unsigned)s->queue_usage,
               (unsigned)s->queue_peak,
               (unsigned long)s->queue_put_count,
               (unsigned long)s->queue_put_timeout);
    logPrintln("[bus] tx    : frames=%lu rate=%lu/s last=%lu us max=%lu us timeout=%lu poll_frames=%lu",
               (unsigned long)s->tx_frames,
               (unsigned long)s->tx_rate_1s,
               (unsigned long)s->tx_time_last_us,
               (unsigned long)s->tx_time_max_us,
               (unsigned long)s->tx_timeout,
               (unsigned long)s->poll_frames);
    logPrintln("[bus] target: wait_max<5ms slow=0, drop<5%% , queue peak<=2, put_timeout=0, urgent max<2ms");
    logPrintln("[bus] usage : step bus | step bus r(clear) | step bus q(quick) | car bus");
#endif /* MOTOR_BUS_STATS */
}

static void Motor_Bus_Stats_Shell(int argc, char *argv[]) {
    const MotorBusStats_t *s = Motor_Bus_Stats_Get();
    bool reset = (argc > 1 && strcmp(argv[1], "r") == 0);
    bool quick = (argc > 1 && strcmp(argv[1], "q") == 0);

    if (reset) {
        Motor_Bus_Stats_Reset();
        logPrintln("[bus] statistics cleared");
        return;
    }

    if (quick) {
        logPrintln("[bus] queue usage=%u peak=%u | motion rate=%luHz drop=%lu wait_max=%lu ms | urgent max=%lu ms",
                   (unsigned)Motor_Get_Queue_Usage(),
                   (unsigned)s->queue_peak,
                   (unsigned long)s->motion_rate_1s,
                   (unsigned long)s->motion_dropped,
                   (unsigned long)s->motion_wait_max_ms,
                   (unsigned long)s->urgent_latency_max_ms);
        return;
    }

    Motor_Bus_Stats_Print();
}

ShellCommand StepGroup[] = {
#if USE_VIEW
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, view, Motor_View_Shell, View Motor Status),
#endif
#if MOTOR_VELOCITY_MODE
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, vel, Motor_vel_Shell, Set Motor Velocity),
#endif
#if MOTOR_POS_MODE
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, pos, Motor_pos_Shell, Set Motor Position),
#endif
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, tool, Motor_tool_Shell, Motor Tools),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, bus, Motor_Bus_Stats_Shell, Motor Bus Chain Stats),
    SHELL_CMD_GROUP_END()};
SHELL_EXPORT_CMD_GROUP(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                       step,
                       StepGroup,
                       Step Control CMD Group);

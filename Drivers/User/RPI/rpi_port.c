/**
 * @file rpi_port.c
 * @author T-Car Team
 * @brief 树莓派(RPi)串口通信与命令封装
 *
 * 分层结构(自下而上):
 *   1) 传输层: USART2 DMA 发送 + 中断接收(逐字节), 原始字节收发;
 *   2) 帧解析层: 定长帧校验、无帧尾应答的帧头定位、[方向位, 幅值] 解码;
 *   3) 命令层: 颜色/数字/校准/完成数量/雷达, 按 rpi_config.h 的协议表实现;
 *   4) 组合层: RPI_Cal(标定 + 输出相对当前位置的修正量)。
 *
 * 单次命令的公共流程(见 RPI_RequestFrame):
 *   确认树莓派已停发 -> 清接收 -> DMA 发送命令 -> 等待匹配格式的应答
 *   -> 补发空闲命令(0x11, 树莓派收到后停止 100ms 周期重复发送);
 *   雷达链路是长会话轮询, 不补发空闲命令(见 RPI_SetGridVal)。
 * 可调参数见 rpi_config.h (禁改)。
 */

#include "rpi_port.h"

#include "usart.h" /* huart2 / MX_USART2_UART_Init */
#include "log.h"
#include "Events.h" /* System_StatusHandle / UART2_* 事件标志 */
#include "cmsis_os2.h"
#include "nav_local.h" /* Loc_Get / PoseTimestamp_t */
#include <math.h> /* cosf / sinf */
#include <string.h> /* memcpy */

#if RPI_RADAR_AVAILABLE == 1
#include "Cstar_map_core.h" /* g_map / set_grid_val / MAP_CELL_DYNAMIC_OBSTACLE */
#endif

/* ==========================================================================
 * 1. 传输层: USART2 DMA 发送 + 中断接收(逐字节)
 *
 * 发送:
 *   HAL_UART_Transmit_DMA 启动 DMA 后阻塞等待发送完成, 返回时机与阻塞式
 *   HAL_UART_Transmit 一致(最后一位数据已移出), 但等待期间任务让出 CPU, 不再是忙等。
 *   完成判据用 HAL 状态(gState 回到 READY 且 TC 置位), 发送完成事件只用于快速唤醒:
 *   ISR 里的 osEventFlagsSet 在 CMSIS-RTOS2 中经定时器守护任务延迟置位, 事件丢失/迟到时
 *   仍能由状态判据兜住, 不会把已经发出去的包误判为发送失败。
 *   DMA 直接读调用方缓冲, 本层等发送完成后才返回, 因此栈上的单字节命令缓冲无需额外拷贝。
 * 接收:
 *   中断逐字节累积到 rpiRxAccum; 只有"等待应答"期间收到的数据进入解析缓冲, 其它
 *   (上一条命令的重复帧/迟到应答)直接丢弃并计数。
 * 停发确认:
 *   每条命令结束后立刻补发 0x11, 但"树莓派是否真的停发"的静默判定推迟到下一条命令前
 *   (RPI_LinkPrepare): 调用方拿到应答后通常还要运动/等待, 这段时间足够树莓派处理 0x11,
 *   停发等待多数情况下没有额外开销。下一条命令前会再补发一次 0x11 并等待
 *   RPI_IDLE_QUIET_MS 静默; 等待超时说明树莓派仍在重复发送, 本条命令直接失败
 *   (下一条命令前重新补发 0x11 再等, 自愈式重试), 避免把上一条命令的残留帧当成应答。
 * ========================================================================== */

/** 中断接收的单字节缓冲 */
static uint8_t s_rx_byte;
/** 应答累积缓冲: 一次命令等待期间收到的所有字节(处理分片/前导垃圾/多帧) */
static uint8_t rpiRxAccum[RPI_RX_ACCUM_SIZE];
/** 累积长度 */
static volatile uint16_t rpiRxAccumLen = 0;
/** 是否正在等待当前命令的应答; 否则收到的数据视为脏数据直接丢弃 */
static volatile bool s_rx_awaiting = false;
/** 中断回调里重装填接收失败(句柄异常)时置位, 由任务侧补装填 */
static volatile bool s_rx_needs_rearm = false;
/** 脏数据统计: 非等待期间收到并丢弃的数据量(字节) */
static volatile uint32_t s_rx_discarded_frames = 0;
static volatile uint32_t s_rx_discarded_bytes = 0;
/** 最近一次收到任意字节(含被丢弃的)的内核节拍: 用于"树莓派是否已停发"的静默判定 */
static volatile uint32_t s_rx_last_tick = 0;
/** 是否正有任务在 RPI_WaitRxQuiet 里等链路活动事件: 否则 ISR 不置位, 避免树莓派持续发送形成事件风暴 */
static volatile bool s_rx_quiet_wait = false;
/** 上次置位 UART2_RX_ACT 的内核节拍: 对持续发送做节流(见 RPI_RX_EVT_THROTTLE_MS) */
static volatile uint32_t s_rx_act_tick = 0;
/** 是否有一次 DMA 发送还没结束(只有超时/错误路径会遗留, 下一次发送前恢复) */
static bool s_tx_busy = false;
/** 上一条命令的 0x11 已发出, 但还没确认树莓派停发(延迟到下一条命令前确认) */
static bool s_stop_pending = false;
/** 颜色识别会话已开始(识别命令已下发, 树莓派正在连续重复上报, 期间不补发 0x11) */
static bool s_color_session = false;
/** 链路互斥: 一条命令是"清接收->发送->等应答->发空闲"的完整事务, 任务/shell 并发会互相踩应答 */
static osMutexId_t s_link_mutex = NULL;
static const osMutexAttr_t s_link_mutex_attr = {
    .name = "RPI_Link",
    .attr_bits = osMutexRecursive,
};

/**
 * @brief 启动/重装填 USART2 中断接收(每次 1 字节)
 * @return true=接收已装填(含"上一次接收仍在进行中"的正常 BUSY 情况)
 * @note HAL_BUSY 表示接收已经装填过(中断里刚装填), 不能当失败反复重试
 */
static bool RPI_RxArm(void) {
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1);

    return (status == HAL_OK) || (status == HAL_BUSY);
}

/**
 * @brief 串口发送完成回调(TC 中断: 最后一位数据已移出)
 * @param huart 串口句柄
 */
static void RPI_UartTxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        osEventFlagsSet(System_StatusHandle, UART2_TX_IDLE);
    }
}

/**
 * @brief 串口接收完成回调(中断模式, 每收 1 字节回调一次)
 * @param huart 串口句柄
 * @note 应答期间把字节追加到累积缓冲, 由任务侧按帧头/帧尾对齐解析;
 *       非应答期间(迟到应答/重复帧)直接丢弃并计数, 绝不带进下一次命令。
 *       每字节只更新时间戳; 事件置位必须"确实有人在等"且做节流 —— ISR 里的
 *       osEventFlagsSet 在 CMSIS-RTOS2 中经 FreeRTOS 定时器命令队列投递
 *       (configTIMER_QUEUE_LENGTH=10, 守护任务优先级 2), 树莓派异常持续发送时
 *       若每字节都置位, 可能挤掉其它模块的 ISR 事件(如 UART6_TX_IDLE)让它们卡死。
 */
static void RPI_UartRxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART2) return;

    uint32_t flags = 0;
    uint32_t now = osKernelGetTickCount();

    if (s_rx_awaiting && ((uint32_t)rpiRxAccumLen + 1U <= RPI_RX_ACCUM_SIZE)) {
        rpiRxAccum[rpiRxAccumLen] = s_rx_byte;
        rpiRxAccumLen = (uint16_t)(rpiRxAccumLen + 1U);
        flags = UART2_RX_CPLT; /* 应答收集者靠它唤醒; 一次应答只有几个字节, 不会形成事件风暴 */
    } else {
        s_rx_discarded_frames++;
        s_rx_discarded_bytes++;
    }

    s_rx_last_tick = now;

    /* 静默判定的真正判据是时间戳, 事件只用于快速唤醒: 只有任务在等且距上次置位超过节流窗口才置位 */
    if (!s_rx_awaiting && s_rx_quiet_wait && ((now - s_rx_act_tick) >= RPI_RX_EVT_THROTTLE_MS)) {
        s_rx_act_tick = now;
        flags |= UART2_RX_ACT;
    }

    if (flags != 0U) {
        osEventFlagsSet(System_StatusHandle, flags);
    }

    /* 立即重装填接收下一字节; 装填失败(句柄异常)时置标志, 由任务侧补装填 */
    if (!RPI_RxArm()) {
        s_rx_needs_rearm = true;
    }
}

/**
 * @brief 串口错误回调: 重新装填接收
 * @param huart 串口句柄
 * @note HAL 已在中断里结束本次接收(清错误标志、RxState 置 READY), 这里只重新装填,
 *       同时重新打开错误中断; 不在中断里强制改写 gState/发送状态: 发送(含 DMA)的错误
 *       由发送等待方超时后统一恢复, 避免与 HAL 正在进行的 DMA 发送流程错位。
 */
static void RPI_UartErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART2) return;

    /* PE/FE/NE/ORE 需要"读 SR + 读 DR"才能清除, 防止重装填后错误中断反复触发 */
    __HAL_UART_CLEAR_PEFLAG(huart);

    if (!RPI_RxArm()) {
        s_rx_needs_rearm = true;
    }
}

/**
 * @brief 等待 DMA 发送完成(最后一位数据已移出)
 * @param timeout_ms 等待超时(ms)
 * @return true=发送完成; false=超时或发送过程出错
 * @note 完成判据不依赖事件: ISR 里的 osEventFlagsSet 在 CMSIS-RTOS2 中要经定时器守护任务
 *       延迟置位, 极端拥塞时可能丢失/迟到, 所以以 HAL 状态为准 —— TC 中断执行时
 *       UART_EndTransmit_IT 先把 gState 置回 READY 再调用发送完成回调, 而 TC 标志在本次
 *       HAL_UART_Transmit_DMA 开始时已被清掉, 因此"gState==READY 且 TC 置位"即本次发送完成。
 *       发送完成事件只用于快速唤醒, 事件丢失最多多等一个 RPI_TX_POLL_MS。
 */
static bool RPI_TxWaitComplete(uint32_t timeout_ms) {
    uint32_t start = osKernelGetTickCount();

    for (;;) {
        if (huart2.gState == HAL_UART_STATE_READY) {
            if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) != RESET) {
                osEventFlagsClear(System_StatusHandle, UART2_TX_IDLE);
                return true;
            }
            /* 句柄已空闲但 TC 未置位: 发送被中止(DMA 错误), 不再等满超时 */
            return false;
        }

        uint32_t elapsed = osKernelGetTickCount() - start;
        if (elapsed >= timeout_ms) {
            break;
        }

        /* 事件只用于尽快唤醒; 最多隔 RPI_TX_POLL_MS 查一次 HAL 状态, 事件延迟/丢失也不会拖满超时 */
        uint32_t wait = timeout_ms - elapsed;
        if (wait > RPI_TX_POLL_MS) {
            wait = RPI_TX_POLL_MS;
        }
        (void)osEventFlagsWait(System_StatusHandle, UART2_TX_IDLE, osFlagsWaitAll, pdMS_TO_TICKS(wait));
    }

    return false;
}

/**
 * @brief 恢复发送状态: 中止未完成的 DMA/中断发送, 把句柄打回 READY
 * @note 只有发送等待超时/启动失败时才需要, 保证下一次 HAL_UART_Transmit_DMA 不会永久 HAL_BUSY
 */
static void RPI_TxRecover(void) {
    (void)HAL_UART_AbortTransmit(&huart2);
    osEventFlagsClear(System_StatusHandle, UART2_TX_IDLE);
    s_tx_busy = false;
}

/**
 * @brief 发送数据到树莓派(DMA 方式, 同步等待发送完成)
 * @param data 数据指针(DMA 直接读取, 本函数返回前不会被改写)
 * @param len  数据长度
 * @return 发送状态
 * @note 单条命令只有 1~几字节, 仍走 DMA: 启动 DMA 后同步等待"最后一位已移出"(判据见
 *       RPI_TxWaitComplete), 返回时机与阻塞式 HAL_UART_Transmit 一致, 但等待期间让出 CPU;
 *       超时/出错则中止发送并恢复句柄, 避免后续发送永久 HAL_BUSY。
 */
static bool RPI_Send(const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U) {
        return false;
    }

    /* 正常路径每次都会等到发送完成; 只有上一次超时/错误才可能遗留 BUSY, 先恢复 */
    if (s_tx_busy || (huart2.gState != HAL_UART_STATE_READY)) {
        logWarning("uart2 tx busy before send, recover");
        RPI_TxRecover();
    }

    osEventFlagsClear(System_StatusHandle, UART2_TX_IDLE);

    if (HAL_UART_Transmit_DMA(&huart2, data, len) != HAL_OK) {
        logWarning("uart2 transmit dma start failed (len=%u)", (unsigned)len);
        RPI_TxRecover();
        return false;
    }
    s_tx_busy = true;

    if (!RPI_TxWaitComplete(RPI_TX_TIMEOUT_MS)) {
        logWarning("uart2 transmit failed (timeout/abort, len=%u)", (unsigned)len);
        RPI_TxRecover();
        return false;
    }

    s_tx_busy = false;
    return true;
}

/**
 * @brief 清空接收状态: 中止当前中断接收并重启, 丢弃残留半帧/迟到数据
 * @note 每次命令发送前调用, 保证之后收到的数据都属于本次命令;
 *       中止失败(RxState 不在 BUSY)时也要重启接收, 否则会永久收不到应答。
 */
static void RPI_FlushRx(void) {
    s_rx_awaiting = false;
    s_rx_needs_rearm = false;
    rpiRxAccumLen = 0;

    (void)HAL_UART_AbortReceive(&huart2);
    /* 中止接收不会清 DR: 丢掉已锁存的残留字节, 避免重装填后把它交给下一次命令 */
    (void)huart2.Instance->DR;
    osEventFlagsClear(System_StatusHandle, UART2_RX_CPLT);
    osEventFlagsClear(System_StatusHandle, UART2_RX_ACT);

    if (!RPI_RxArm()) {
        s_rx_needs_rearm = true;
        logWarning("uart2 rx arm failed");
    }
}

/**
 * @brief 等待接收链路静默指定时长
 * @param quiet_ms 需要连续静默的时长(ms)
 * @param max_ms   最长等待时间(ms)
 * @return true=已静默 quiet_ms(期间没有新字节); false=等待超时
 * @note 不轮询固定间隔: 用 UART2_RX_ACT 事件唤醒, 每收到一个字节就重新计时。
 *       事件只是"快速唤醒"用途(受 RPI_RX_EVT_THROTTLE_MS 节流), 真正的判据是
 *       s_rx_last_tick 时间戳, 因此事件迟到/丢失都不会让本函数误判静默;
 *       若字节在"读时间戳之后、清标志之前"到达, 本轮等待会被唤醒并重新计时;
 *       若标志被清空后才置位, 最多多等一个窗口, 超时后回到循环顶部按最新时间戳
 *       再判定一次兜底, 不会把"仍在发送"误判为静默。
 */
static bool RPI_WaitRxQuiet(uint32_t quiet_ms, uint32_t max_ms) {
    uint32_t start = osKernelGetTickCount();

    /* 进入等待才允许 RX ISR 置位链路活动事件(并节流), 退出时恢复: 链路空闲/持续发送时不产生事件开销 */
    s_rx_quiet_wait = true;
    s_rx_act_tick = start - RPI_RX_EVT_THROTTLE_MS; /* 允许第一个字节立即置位 */

    for (;;) {
        uint32_t now = osKernelGetTickCount();
        uint32_t idle = now - s_rx_last_tick;

        if (idle >= quiet_ms) {
            s_rx_quiet_wait = false;
            return true;
        }
        if ((now - start) >= max_ms) {
            s_rx_quiet_wait = false;
            return false;
        }

        uint32_t wait = quiet_ms - idle;
        uint32_t remain = max_ms - (now - start);
        if (wait > remain) {
            wait = remain;
        }

        osEventFlagsClear(System_StatusHandle, UART2_RX_ACT);
        uint32_t flags = osEventFlagsWait(System_StatusHandle, UART2_RX_ACT, osFlagsWaitAll, pdMS_TO_TICKS(wait));
        if ((flags & osFlagsError) != 0U) {
            /* 本窗口内没有新数据: 回到循环顶部按最新时间戳再确认一次 */
            continue;
        }
    }
}

/**
 * @brief 补发空闲命令 0x11, 让树莓派退出当前命令状态、停止重复发送
 * @note 只负责把 0x11 发出去(DMA 完成), "是否真的停发"由下一条命令前的
 *       RPI_LinkPrepare 确认, 因此调用方收到应答后可以立刻返回, 不必每条命令都等满静默窗口;
 *       即使发送失败也把 s_stop_pending 置位: 0x11 可能没到树莓派, 下一次链路准备必须
 *       重新补发并确认停发, 否则可能把上一条命令的重复帧当成新应答;
 *       雷达长会话不补发空闲命令(树莓派自己以 0xEF 结束会话)。
 */
static void RPI_SendIdle(void) {
    uint8_t cmd = RPI_CMD_IDLE;

    (void)RPI_Send(&cmd, sizeof(cmd));
    s_stop_pending = true;
}

/**
 * @brief 下一条命令前的链路准备: 确认树莓派已停发上一命令的重复帧 + 清空接收
 * @return true=链路已就绪(可以发下一条命令); false=树莓派仍在重复发送, 本条命令应放弃
 * @note 停发确认基于"最后一次收到字节"的时间戳: 命令间隔通常远大于 RPI_IDLE_QUIET_MS,
 *       此时这里几乎零等待 —— 等于把原来每条命令后固定 150ms 的停发等待挪出了关键路径。
 *       等待前再补发一次 0x11: 上一条命令末尾的 0x11 可能因发送失败/丢失没有到达树莓派,
 *       补发成本只有 1 字节, 却能保证它一定收到过空闲命令。
 *       等待超时说明树莓派仍在重复发送(链路异常/树莓派卡住), 若继续发新命令, 上一条命令的
 *       重复帧可能被当成本次应答(0x0A/0x0D 都是 6 字节负载, 同格式时无法区分), 所以直接
 *       判定本次命令失败; 保留 s_stop_pending 让下一条命令前继续补发 0x11 并重试(自愈式)。
 */
static bool RPI_LinkPrepare(void) {
    if (!s_stop_pending && !s_color_session) {
        RPI_FlushRx();
        return true;
    }

    /* 颜色识别会话进行中: 其它命令要用链路, 先结束会话(本次补发的 0x11 让树莓派停发,
     * 停发确认由下面的静默等待完成) */
    if (s_color_session) {
        s_color_session = false;
        s_rx_awaiting = false;
    }

    RPI_SendIdle(); /* 补发 0x11, 同时把 s_stop_pending 重新置位 */

    if (!RPI_WaitRxQuiet(RPI_IDLE_QUIET_MS, RPI_IDLE_MAX_WAIT_MS)) {
        logWarning("rpi still repeating (%u ms), link not ready", (unsigned)RPI_IDLE_MAX_WAIT_MS);
        RPI_FlushRx();
        return false; /* 保留 s_stop_pending: 下一条命令前继续补发 0x11 并等待 */
    }

    s_stop_pending = false;
    RPI_FlushRx();
    return true;
}

/**
 * @brief 获取链路互斥(递归锁: 组合接口内部再调用命令接口也不会自锁; 创建失败时退化为无锁)
 * @return true=获取成功
 */
static bool RPI_LinkLock(void) {
    if (s_link_mutex == NULL) {
        return true;
    }

    return osMutexAcquire(s_link_mutex, pdMS_TO_TICKS(RPI_LOCK_TIMEOUT_MS)) == osOK;
}

/** @brief 释放链路互斥 */
static void RPI_LinkUnlock(void) {
    if (s_link_mutex != NULL) {
        (void)osMutexRelease(s_link_mutex);
    }
}

/**
 * @brief 初始化树莓派通信(USART2 DMA 发送 + 中断接收 + 链路互斥)
 * @note 初始化末尾会补发一次空闲命令: 上电/复位时树莓派可能还停在上一次命令状态
 *       (持续重复发送), 由第一条命令前的 RPI_LinkPrepare 负责确认其已停发。
 */
bool RPI_Init(void) {
    MX_USART2_UART_Init();

    s_rx_last_tick = osKernelGetTickCount();
    s_rx_quiet_wait = false;
    s_rx_act_tick = 0;
    s_tx_busy = false;
    s_stop_pending = false;
    s_rx_awaiting = false;
    s_rx_needs_rearm = false;
    rpiRxAccumLen = 0;

    osEventFlagsClear(System_StatusHandle, UART2_TX_IDLE);
    osEventFlagsClear(System_StatusHandle, UART2_RX_CPLT);
    osEventFlagsClear(System_StatusHandle, UART2_RX_ACT);

    /* 链路互斥创建失败只降级为无保护(不影响通信本身), 不阻断模块初始化 */
    if (s_link_mutex == NULL) {
        s_link_mutex = osMutexNew(&s_link_mutex_attr);
        if (s_link_mutex == NULL) {
            logWarning("rpi link mutex create failed");
        }
    }

    /* 回调必须先于装填接收/首次发送注册: 装填后随时可能收到字节或完成发送,
       回调未注册时数据会落到弱定义的空实现上, 被静默丢弃 */
    if (HAL_UART_RegisterCallback(&huart2, HAL_UART_TX_COMPLETE_CB_ID, RPI_UartTxCpltCallback) != HAL_OK) {
        logWarning("Failed to register tx callback for USART2");
        return false;
    }

    if (HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, RPI_UartRxCpltCallback) != HAL_OK) {
        logWarning("Failed to register rx callback for USART2");
        return false;
    }

    if (HAL_UART_RegisterCallback(&huart2, HAL_UART_ERROR_CB_ID, RPI_UartErrorCallback) != HAL_OK) {
        logWarning("Failed to register error callback for USART2");
        return false;
    }

    if (!RPI_RxArm()) {
        logWarning("uart2 rx arm failed");
        return false;
    }

    /* 上电/复位时树莓派可能还停在上一次命令状态(持续重复发送): 补发空闲命令,
       由第一条命令前的 RPI_LinkPrepare 确认其已停发 */
    RPI_FlushRx();
    RPI_SendIdle();

    return true;
}

/** 应答帧匹配类型: 只接受与当前命令格式一致的帧, 其它(迟到/串扰帧)丢弃继续等 */
typedef enum {
    RPI_RSP_EXPECT_FIXED = 0, // 定长帧 [0xFE, payload_len, 0xEF]
    RPI_RSP_EXPECT_COLOR, // 颜色帧 [0xFE, color, 0xEF] 或早期相反顺序 [0xEF, color, 0xFE]
    RPI_RSP_EXPECT_LENIENT, // 无帧尾帧 [0xFE, payload_len...]
} RpiExpect_t;

/**
 * @brief 检查接收累积缓冲里是否已有与期望格式匹配的完整应答帧
 * @param payload_len 期望负载长度(validate 模式)
 * @param expect      期望帧格式
 * @param frame_ofs   输出: 匹配到的帧头在累积缓冲里的偏移(允许前导垃圾)
 * @return true=找到匹配帧
 * @note 不加日志, 供等待循环反复调用; 帧格式不匹配的残留/串扰帧不算应答
 */
static bool RPI_AccumHasExpected(uint16_t payload_len, RpiExpect_t expect, uint16_t *frame_ofs) {
    const uint8_t *buf = rpiRxAccum;
    uint16_t len = rpiRxAccumLen;

    switch (expect) {
    case RPI_RSP_EXPECT_COLOR:
        for (uint16_t i = 0; (uint16_t)(i + RPI_FRAME_LEN(RPI_PAYLOAD_LEN_COLOR)) <= len; i++) {
            uint8_t head = buf[i];
            uint8_t value = buf[i + 1];
            uint8_t tail = buf[i + 2];
            bool normal_frame = (head == RPI_COLOR_FRAME_HEAD && tail == RPI_COLOR_FRAME_TAIL);
            bool alt_frame = (head == RPI_COLOR_FRAME_HEAD_ALT && tail == RPI_COLOR_FRAME_TAIL_ALT);
            if ((normal_frame || alt_frame) && value <= COLOR_LIGHTBLUE) {
                *frame_ofs = i;
                return true;
            }
        }
        return false;

    case RPI_RSP_EXPECT_FIXED: {
        uint16_t frame_len = RPI_FRAME_LEN(payload_len);
        for (uint16_t i = 0; (uint16_t)(i + frame_len) <= len; i++) {
            if (buf[i] == RPI_FRAME_HEAD && buf[i + RPI_FRAME_TAIL_IDX(payload_len)] == RPI_FRAME_TAIL) {
                *frame_ofs = i;
                return true;
            }
        }
        return false;
    }

    default: /* RPI_RSP_EXPECT_LENIENT */
        for (uint16_t i = 0; (uint16_t)(i + RPI_FRAME_MIN_LEN(payload_len)) <= len; i++) {
            if (buf[i] == RPI_FRAME_HEAD) {
                *frame_ofs = i;
                return true;
            }
        }
        return false;
    }
}

/**
 * @brief 把累积缓冲从指定偏移拷出到调用方缓冲(超出容量部分截断)
 * @param rsp  目标缓冲
 * @param size 目标容量
 * @param ofs  起始偏移(匹配到的帧头位置)
 * @return 实际拷贝的字节数
 */
static uint16_t RPI_CopyAccumFrom(uint8_t *rsp, uint16_t size, uint16_t ofs) {
    uint16_t len = (ofs < rpiRxAccumLen) ? (uint16_t)(rpiRxAccumLen - ofs) : 0U;

    if (len > size) {
        len = size;
    }
    if (len > 0) {
        memcpy(rsp, &rpiRxAccum[ofs], len);
    }
    return len;
}

/**
 * @brief 从累积缓冲头部拷出(裸模式未做帧匹配时使用)
 */
static uint16_t RPI_CopyAccum(uint8_t *rsp, uint16_t size) {
    return RPI_CopyAccumFrom(rsp, size, 0U);
}

/**
 * @brief 等待本次命令的应答字节
 * @param rsp         应答缓冲
 * @param size        应答缓冲容量
 * @param timeout_ms  总超时
 * @param payload_len 期望负载长度(validate=true 时有效)
 * @param expect      期望帧格式(validate=true 时有效)
 * @param validate    true=只接受与期望格式匹配的帧(不匹配的丢弃继续等); false=收到任何数据即返回
 * @return 返回的字节数(validate=true 时从匹配帧头开始, 不带前导垃圾; validate=false 时为原始接收字节);
 *         0=超时或没有匹配帧
 * @note 调用前必须已置位 s_rx_awaiting; 返回后由调用方清除。
 */
static uint16_t RPI_CollectResponse(
    uint8_t *rsp, uint16_t size, uint32_t timeout_ms, uint16_t payload_len, RpiExpect_t expect, bool validate) {
    uint32_t start = osKernelGetTickCount();

    for (;;) {
        /* 中断回调里重装填失败(句柄异常)时, 由任务侧补装填, 避免接收静默停摆 */
        if (s_rx_needs_rearm && RPI_RxArm()) {
            s_rx_needs_rearm = false;
        }

        if (validate) {
            uint16_t frame_ofs = 0;
            if (RPI_AccumHasExpected(payload_len, expect, &frame_ofs)) {
                return RPI_CopyAccumFrom(rsp, size, frame_ofs);
            }
        }

        uint32_t elapsed = osKernelGetTickCount() - start;
        if (elapsed >= timeout_ms) {
            break;
        }

        uint32_t wait = timeout_ms - elapsed;
        if (rpiRxAccumLen > 0 && wait > RPI_RX_QUIET_MS) {
            /* 已有数据: 用静默窗口判断这一帧是否收完(抵抗帧被拆成多次中断交付) */
            wait = RPI_RX_QUIET_MS;
        }

        uint32_t flags = osEventFlagsWait(System_StatusHandle, UART2_RX_CPLT, osFlagsWaitAll, pdMS_TO_TICKS(wait));
        if ((flags & osFlagsError) == 0U) {
            /* 有新数据: 清标志后回到循环顶部检查/继续收 */
            osEventFlagsClear(System_StatusHandle, UART2_RX_CPLT);
            continue;
        }

        if (rpiRxAccumLen == 0) {
            continue; /* 还没收到任何数据: 继续等剩余超时 */
        }

        if (!validate) {
            return RPI_CopyAccum(rsp, size); /* 裸模式: 数据已静默, 收完 */
        }

        /* 校验模式: 本批数据里没有匹配帧 = 其它命令的重复帧/残留;
         * 丢弃后继续等到总超时, 只接受与当前命令格式一致的应答
         * (树莓派每 100ms 重复发送, 正确帧可能比残留帧晚到) */
        rpiRxAccumLen = 0;
        continue;
    }

    return 0;
}

#if RPI_RADAR_AVAILABLE == 1
/**
 * @brief 发送单字节命令并等待原始应答(不做帧格式校验, 供雷达长会话使用)
 * @param cmd  命令字节
 * @param rsp  应答缓冲区
 * @param size 应答缓冲区容量(接收上限)
 * @return 实际接收到的字节数, 超时/错误返回 0
 * @note 发送为 DMA + 完成事件, 接收为中断模式(逐字节), 通过 UART2_RX_CPLT 通知;
 *       收发前统一做链路准备(RPI_LinkPrepare: 等上一命令停发 + 清接收), 避免迟到应答/
 *       残留半帧被当成本次应答; 本函数只负责收发并返回实际长度, 不做完整性判定、
 *       不补发空闲命令(雷达长会话由树莓派自己以 0xEF 结束)
 */
static uint16_t RPI_Send_And_Receive(uint8_t cmd, uint8_t *rsp, uint16_t size) {
    if (rsp == NULL || size == 0) return 0;

    if (!RPI_LinkLock()) {
        logWarning("rpi link busy (lock timeout)");
        return 0;
    }

    if (!RPI_LinkPrepare()) {
        RPI_LinkUnlock(); /* 原因已由 RPI_LinkPrepare 记录, 这里不重复打日志 */
        return 0;
    }
    s_rx_awaiting = true;

    uint16_t len = 0;
    if (RPI_Send(&cmd, sizeof(cmd))) {
        len = RPI_CollectResponse(rsp, size, RPI_RSP_TIMEOUT_MS, 0, RPI_RSP_EXPECT_FIXED, false);
        if (len == 0) {
            logWarning("receive timeout");
        }
    }

    s_rx_awaiting = false;
    rpiRxAccumLen = 0;

    RPI_LinkUnlock();
    return len;
}
#endif /* RPI_RADAR_AVAILABLE */

/**
 * @brief 单次命令交互: 链路准备 -> DMA 发送命令 -> 等待匹配格式的应答 -> 补发空闲命令
 * @param cmd         命令字节
 * @param rsp         应答缓冲区
 * @param size        应答缓冲区容量
 * @param payload_len 期望负载长度
 * @param expect      期望应答帧格式
 * @return 实际接收到的字节数(0=超时/无匹配帧)
 * @note 树莓派收到命令字节即切换命令状态, 无需先发空闲命令;
 *       它会在主循环里每 100ms 重复发送同一应答, 因此收到应答后补发空闲命令(0x11);
 *       停发确认推迟到下一条命令前(RPI_LinkPrepare), 调用方拿到结果后可立即返回;
 *       等待应答期间只接受与 payload_len/expect 匹配的帧, 其它残留/串扰帧丢弃并继续等待;
 *       链路准备失败(树莓派仍在重复发送)时本次命令直接返回失败, 由上层重试。
 */
static uint16_t RPI_RequestFrame(uint8_t cmd, uint8_t *rsp, uint16_t size, uint16_t payload_len, RpiExpect_t expect) {
    if (rsp == NULL || size == 0) return 0;

    if (!RPI_LinkLock()) {
        logWarning("rpi link busy (lock timeout)");
        return 0;
    }

    if (!RPI_LinkPrepare()) {
        RPI_LinkUnlock(); /* 原因已由 RPI_LinkPrepare 记录, 这里不重复打日志 */
        return 0;
    }
    s_rx_awaiting = true;

    uint16_t len = 0;
    if (RPI_Send(&cmd, sizeof(cmd))) {
        len = RPI_CollectResponse(rsp, size, RPI_RSP_TIMEOUT_MS, payload_len, expect, true);
        if (len == 0) {
            logWarning("response timeout/frame mismatch (cmd 0x%02X, expect payload %u)", cmd, (unsigned)payload_len);
        }
    }

    s_rx_awaiting = false;
    rpiRxAccumLen = 0;

    /* 收到应答(或超时)后补发 0x11 让树莓派停发; 停发确认交给下一条命令前的链路准备,
     * 不占用本条命令的关键路径, 也避免残留重复帧污染下一条命令(同格式命令时尤其重要) */
    RPI_SendIdle();

    RPI_LinkUnlock();
    return len;
}

/**
 * @brief 直接发送原始字节并等待原始应答(调试用)
 * @param data       待发送字节
 * @param len        发送长度
 * @param rsp        应答缓冲区, 传 NULL 表示只发不收
 * @param size       应答缓冲区容量
 * @param timeout_ms 等待应答超时(ms)
 * @return 实际接收到的字节数, 0 表示无应答
 * @note 供 shell 手工调试; 与命令路径一致: 先做链路准备(树莓派未停发则本次直接放弃),
 *       收发后补发空闲命令 0x11, 避免调试时把树莓派留在上一个命令状态而污染后续命令的应答;
 *       rsp=NULL(只发不收)时, 补发空闲命令前会等 RPI_RAW_IDLE_DELAY_MS:
 *       树莓派接收线程按批读取并只取最后一条有效命令, 紧接着发 0x11 会把原始命令覆盖掉。
 */
uint16_t RPI_Send_Raw(const uint8_t *data, uint16_t len, uint8_t *rsp, uint16_t size, uint32_t timeout_ms) {
    if (data == NULL || len == 0) return 0;

    if (!RPI_LinkLock()) {
        logWarning("rpi link busy (lock timeout)");
        return 0;
    }

    if (!RPI_LinkPrepare()) {
        RPI_LinkUnlock(); /* 原因已由 RPI_LinkPrepare 记录, 这里不重复打日志 */
        return 0;
    }
    s_rx_awaiting = (rsp != NULL && size > 0);

    uint16_t n = 0;
    if (RPI_Send(data, len)) {
        if (s_rx_awaiting) {
            n = RPI_CollectResponse(rsp, size, timeout_ms, 0, RPI_RSP_EXPECT_FIXED, false);
        } else {
            /* 只发不收: 先让树莓派把原始命令读走, 再补发空闲命令, 避免同批数据被覆盖 */
            osDelay(RPI_RAW_IDLE_DELAY_MS);
        }
    }

    s_rx_awaiting = false;
    rpiRxAccumLen = 0;

    /* 与命令路径一致: 补发空闲命令, 避免树莓派继续重复发送本帧污染后续命令 */
    RPI_SendIdle();

    RPI_LinkUnlock();
    return n;
}

/* ==========================================================================
 * 2. 帧解析层: 帧校验与字段解码
 * ========================================================================== */

/**
 * @brief 校验定长应答帧, 返回负载首字节
 * @param rsp         接收缓冲区
 * @param len         实际接收长度
 * @param payload_len 期望负载长度
 * @return 负载首字节指针; 帧不完整/定界错误返回 NULL
 * @note 定长帧布局: [0xFE, 负载(payload_len), 0xEF];
 *       在接收缓冲里扫描"帧头 + 预期位置的帧尾"完全对齐的一帧, 允许前面有前导垃圾
 *       (迟到应答残留/树莓派调试输出/半帧), 不再要求 rsp[0] 就是帧头。
 */
static const uint8_t *RPI_Frame_Payload(const uint8_t *rsp, uint16_t len, uint16_t payload_len) {
    uint16_t frame_len = RPI_FRAME_LEN(payload_len);

    for (uint16_t i = 0; (uint16_t)(i + frame_len) <= len; i++) {
        if (rsp[i] == RPI_FRAME_HEAD && rsp[i + RPI_FRAME_TAIL_IDX(payload_len)] == RPI_FRAME_TAIL) {
            return &rsp[i + 1];
        }
    }

    logWarning("frame head/tail not found (len=%u, payload=%u)", (unsigned)len, (unsigned)payload_len);
    return NULL;
}

#if RPI_COMPLETENUM_ENABLED == 1 || RPI_RADAR_AVAILABLE == 1
/**
 * @brief 在"无固定帧尾"应答(0x0F/0x1F)中自后向前定位帧头, 返回负载首字节
 * @param rsp         接收缓冲区
 * @param len         实际接收长度
 * @param payload_len 负载长度
 * @return 负载首字节指针; 长度不足/未找到帧头返回 NULL
 * @note 上界取 len - 最小帧长, 保证帧头之后负载完整
 *       (否则会解析到未收到的填充字节, 得到 0 值并静默写入);
 *       自后向前定位可容忍接收缓冲里多帧/前导垃圾(取最后一帧)。
 */
static const uint8_t *RPI_Frame_FindPayload(const uint8_t *rsp, uint16_t len, uint16_t payload_len) {
    uint16_t min_len = RPI_FRAME_MIN_LEN(payload_len);

    if (len < min_len) {
        return NULL;
    }

    for (uint16_t i = (uint16_t)(len - min_len + 1); i-- > 0;) {
        if (rsp[i] == RPI_FRAME_HEAD) {
            return &rsp[i + 1];
        }
    }

    logWarning("frame head not found");
    return NULL;
}
#endif /* RPI_COMPLETENUM_ENABLED || RPI_RADAR_AVAILABLE */

/**
 * @brief 解码 [方向位, 幅值] 字段对(约定: dir=0 为正, 非 0 为负)
 * @param dir 方向位: 0 = 正, 非 0 = 负
 * @param mag 幅值
 * @return 带符号值
 * @note 该约定只用于树莓派脚本按"0=正"输出的字段(如圆环 dir_y / 未实现的完成数量字段)。
 */
static int16_t RPI_DecodeSignedZeroIsPos(uint8_t dir, uint8_t mag) {
    return (dir == 0) ? (int16_t)mag : -(int16_t)mag;
}

/**
 * @brief 解码 [方向位, 幅值] 字段对(约定: dir=1 为正, 0 为负)
 * @param dir 方向位: 非 0 = 正, 0 = 负
 * @param mag 幅值
 * @return 带符号值
 * @note 树莓派实际脚本(1.py)部分字段用该约定, 与旧协议文档相反, 必须按实际脚本解码:
 *       - 圆盘 0x0C: dir_x / dir_y 均为 "1=正, 0=负";
 *       - 圆环 0x0A/0x0D: dir_x / dir_angle 为 "1=正, 0=负"。
 *       若树莓派脚本修正了符号位约定, 改回 RPI_DecodeSignedZeroIsPos 即可。
 */
static int16_t RPI_DecodeSignedOneIsPos(uint8_t dir, uint8_t mag) {
    return (dir != 0) ? (int16_t)mag : -(int16_t)mag;
}

/* ==========================================================================
 * 3. 命令层: 按 rpi_config.h 的协议表实现各命令
 * ========================================================================== */

/**
 * @brief 从接收缓冲扫描颜色帧并解码
 * @param buf   接收数据(可能带前导垃圾/残留帧)
 * @param len   数据长度
 * @param color 输出: 检测到的颜色
 * @return true=找到合法颜色帧
 */
static bool rpi_color_frame_parse(const uint8_t *buf, uint16_t len, RPI_Color_t *color) {
    /* 在接收缓冲里扫描颜色帧([0xFE,color,0xEF] 实际格式 或 [0xEF,color,0xFE] 旧格式),
     * 允许前面有前导垃圾(RPI 重复帧残留/调试输出), 避免一个脏字节就判定失败 */
    uint16_t frame_len = RPI_FRAME_LEN(RPI_PAYLOAD_LEN_COLOR);
    for (uint16_t i = 0; (uint16_t)(i + frame_len) <= len; i++) {
        uint8_t head = buf[i];
        uint8_t value = buf[i + 1];
        uint8_t tail = buf[i + 2];

        bool normal_frame = (head == RPI_COLOR_FRAME_HEAD && tail == RPI_COLOR_FRAME_TAIL);
        bool alt_frame = (head == RPI_COLOR_FRAME_HEAD_ALT && tail == RPI_COLOR_FRAME_TAIL_ALT);
        if (!normal_frame && !alt_frame) {
            continue;
        }
        if (value > COLOR_LIGHTBLUE) {
            logWarning("invalid color value: %d", value);
            return false;
        }

        *color = (RPI_Color_t)value;
        return true;
    }

    logWarning("color frame not found (len=%u)", (unsigned)len);
    return false;
}

/**
 * @brief 树莓派检测物料颜色(单色)
 * @param color 输出的颜色
 * @return 解析状态
 * @note 树莓派实际脚本(1.py)回传 [0xFE, color, 0xEF](3 字节, 与通用帧相同);
 *       同时兼容早期脚本/协议用过的相反定界顺序 [0xEF, color, 0xFE]。
 */
bool RPI_Detect_Color(RPI_Color_t *color) {
    if (color == NULL) return false;

    uint8_t rsp[RPI_FRAME_LEN(RPI_PAYLOAD_LEN_COLOR)] = {0};
    uint16_t len =
        RPI_RequestFrame(RPI_CMD_DETECT_COLOR, rsp, sizeof(rsp), RPI_PAYLOAD_LEN_COLOR, RPI_RSP_EXPECT_COLOR);
    if (len < RPI_FRAME_LEN(RPI_PAYLOAD_LEN_COLOR)) {
        logWarning("color rsp too short: %u", (unsigned)len);
        return false;
    }

    return rpi_color_frame_parse(rsp, len, color);
}

/* ==========================================================================
 * 颜色识别会话: 抓取期间保持树莓派连续上报颜色(不重复下发识别命令/不补 0x11)
 * ========================================================================== */

/**
 * @brief 开始颜色识别会话: 下发一次颜色识别命令, 之后不补发 0x11(树莓派保持连续重复上报)
 * @return true=识别命令已下发(或会话已在进行); false=链路忙/链路准备失败/发送失败
 * @note 与 RPI_Detect_Color 的区别: 单次命令接口收到一帧后立刻补发 0x11 让树莓派停发,
 *       下次采样必须重新下发识别命令(颜色锁定约需 2s); 会话接口只在开始时下发一次命令,
 *       抓取期间用 RPI_ColorSessionRead 连续采样(约 100ms 一帧), 抓完当前颜色后再
 *       RPI_ColorSessionEnd 补发 0x11, 避免每次采样都重新颜色锁定, 明显缩短抓取时间。
 */
bool RPI_ColorSessionBegin(void) {
    if (s_color_session) {
        return true;
    }

    if (!RPI_LinkLock()) {
        logWarning("rpi link busy (lock timeout)");
        return false;
    }

    bool ok = false;
    if (RPI_LinkPrepare()) {
        uint8_t cmd = RPI_CMD_DETECT_COLOR;

        rpiRxAccumLen = 0;
        s_rx_awaiting = false; /* 采样前的重复帧直接丢弃, 由各次 Read 等待新帧 */
        if (RPI_Send(&cmd, sizeof(cmd))) {
            s_color_session = true; /* 关键: 这里不补发 0x11, 树莓派会保持连续上报 */
            ok = true;
        }
    }

    RPI_LinkUnlock();
    return ok;
}

/**
 * @brief 颜色识别会话中读取下一帧颜色(不重新下发识别命令, 也不补 0x11)
 * @param color      输出: 检测到的颜色
 * @param timeout_ms 等待下一帧的超时(ms); 首帧等颜色锁定用 RPI_RSP_TIMEOUT_MS, 后续帧用 RPI_COLOR_SAMPLE_TIMEOUT_MS
 * @return true=读到一帧颜色; false=会话未开始/链路忙/超时
 */
bool RPI_ColorSessionRead(RPI_Color_t *color, uint32_t timeout_ms) {
    if (!s_color_session || color == NULL) {
        return false;
    }

    if (!RPI_LinkLock()) {
        logWarning("rpi link busy (lock timeout)");
        return false;
    }

    uint8_t rsp[RPI_FRAME_LEN(RPI_PAYLOAD_LEN_COLOR)] = {0};
    rpiRxAccumLen = 0;
    s_rx_awaiting = true;
    uint16_t len = RPI_CollectResponse(rsp, sizeof(rsp), timeout_ms, RPI_PAYLOAD_LEN_COLOR, RPI_RSP_EXPECT_COLOR, true);
    s_rx_awaiting = false;
    rpiRxAccumLen = 0;

    RPI_LinkUnlock();

    if (len == 0) {
        return false;
    }
    return rpi_color_frame_parse(rsp, len, color);
}

/**
 * @brief 结束颜色识别会话: 补发 0x11 让树莓派停止连续上报
 * @note 抓完当前目标颜色后调用; 之后下一次抓取用 RPI_ColorSessionBegin 重新下发识别命令。
 */
void RPI_ColorSessionEnd(void) {
    if (!s_color_session) {
        return;
    }
    s_color_session = false;

    if (!RPI_LinkLock()) {
        /* 拿不到锁: 让下一条命令的链路准备补发 0x11 并确认停发, 避免树莓派一直重复上报 */
        s_stop_pending = true;
        logWarning("rpi color session end: link busy, idle will be sent by next command");
        return;
    }

    s_rx_awaiting = false;
    rpiRxAccumLen = 0;
    RPI_SendIdle();
    RPI_LinkUnlock();
}

/**
 * @brief 树莓派圆环内数字检测
 * @param num 输出的圆环内数字
 * @return 检测状态
 */
bool RPI_Detect_NUM(uint8_t *num) {
    if (num == NULL) return false;

    uint8_t rsp[RPI_FRAME_LEN(RPI_PAYLOAD_LEN_NUM)] = {0};
    uint16_t len = RPI_RequestFrame(RPI_CMD_DETECT_NUM, rsp, sizeof(rsp), RPI_PAYLOAD_LEN_NUM, RPI_RSP_EXPECT_FIXED);

    const uint8_t *pl = RPI_Frame_Payload(rsp, len, RPI_PAYLOAD_LEN_NUM);
    if (pl == NULL) {
        return false;
    }

    *num = pl[0];

    return true;
}

/**
 * @brief 标定命令描述: 命令字 + 应答负载长度 + 是否携带朝向字段
 */
typedef struct {
    uint8_t cmd;
    uint16_t payload_len;
    bool has_angle;
} RPI_CalCmd_t;

/** 标定类型 -> 命令/应答帧 描述表(索引为 RPI_Cal_t) */
static const RPI_CalCmd_t s_cal_cmds[CAL_NONE] = {
    [CAL_DISK] = {.cmd = RPI_CMD_CAL_DISK, .payload_len = RPI_PAYLOAD_LEN_CAL_DISK, .has_angle = false},
    [CAL_RING_FIRST] = {.cmd = RPI_CMD_CAL_RING_FIRST, .payload_len = RPI_PAYLOAD_LEN_CAL_RING, .has_angle = true},
    [CAL_RING_SECOND] = {.cmd = RPI_CMD_CAL_RING_SECOND, .payload_len = RPI_PAYLOAD_LEN_CAL_RING, .has_angle = true},
    [CAL_STORAGE] = {.cmd = RPI_CMD_CAL_STORAGE, .payload_len = RPI_PAYLOAD_LEN_CAL_RING, .has_angle = true},
};

uint8_t RPI_CalCmd(RPI_Cal_t type) {
    if ((unsigned int)type >= (unsigned int)CAL_NONE) {
        return 0;
    }
    return s_cal_cmds[type].cmd;
}

/**
 * @brief 树莓派偏移/朝向校准（圆盘/圆环）
 * @param err_x         x 轴误差 mm(带符号)
 * @param err_y         y 轴误差 mm(带符号)
 * @param err_angle_deg 朝向误差 deg(带符号); 圆盘标定无该字段, 输出 0
 * @param type          校准类型
 * @return 校准状态
 */
bool RPI_Calibrate(int16_t *err_x, int16_t *err_y, float *err_angle_deg, RPI_Cal_t type) {
    if (err_x == NULL || err_y == NULL || err_angle_deg == NULL) {
        return false;
    }

    if ((unsigned int)type >= (unsigned int)CAL_NONE) {
        logWarning("invalid cal type: %d", type);
        return false;
    }

    const RPI_CalCmd_t *cal = &s_cal_cmds[type];

    /* 缓冲区按最长帧(圆环带角度)开; 圆盘应答更短, 实际帧长由描述表决定 */
    uint8_t rsp[RPI_FRAME_LEN(RPI_PAYLOAD_LEN_CAL_RING)] = {0};
    uint16_t len = RPI_RequestFrame(cal->cmd, rsp, sizeof(rsp), cal->payload_len, RPI_RSP_EXPECT_FIXED);

    const uint8_t *pl = RPI_Frame_Payload(rsp, len, cal->payload_len);
    if (pl == NULL) {
        return false;
    }

    /* 按树莓派实际脚本的符号位解码后, 再按 RPI_CAL_SIGN_* 做坐标方向校准(默认 1.0=不翻转) */
    *err_x = (int16_t)((float)RPI_DecodeSignedOneIsPos(pl[RPI_CAL_IDX_DIR_X], pl[RPI_CAL_IDX_ABS_DX]) * RPI_CAL_SIGN_X);

    if (cal->has_angle) {
        /* 圆环(0x0A/0x0D): 实际脚本 dir_y 为 "0=正", dir_x/dir_angle 为 "1=正" */
        int16_t y_err = RPI_DecodeSignedZeroIsPos(pl[RPI_CAL_IDX_DIR_Y], pl[RPI_CAL_IDX_ABS_DY]);
        int16_t angle_deci_deg = RPI_DecodeSignedOneIsPos(pl[RPI_CAL_IDX_DIR_ANGLE], pl[RPI_CAL_IDX_ABS_ANGLE]);
        *err_y = (int16_t)((float)y_err * RPI_CAL_SIGN_Y);
        *err_angle_deg = (float)angle_deci_deg * RPI_CAL_SIGN_ANGLE / RPI_CAL_ANGLE_SCALE;
    } else {
        /* 圆盘(0x0C): 实际脚本 dir_x/dir_y 都是 "1=正, 0=负", 无角度字段 */
        *err_y =
            (int16_t)((float)RPI_DecodeSignedOneIsPos(pl[RPI_CAL_IDX_DIR_Y], pl[RPI_CAL_IDX_ABS_DY]) * RPI_CAL_SIGN_Y);
        *err_angle_deg = 0.0f;
    }

    return true;
}

/* ==========================================================================
 * 4. 组合层: 标定 + 输出相对当前位置的修正量
 * ========================================================================== */

/**
 * @brief 树莓派校准: 下发视觉标定命令并输出"相对当前位置的视觉修正量"
 * @param name            目标点名称(按 nav_map 查表, 用于日志/取预设)
 * @param offset_x_mm     目标点机体系 X 固定偏移(mm, +前)
 * @param offset_y_mm     目标点机体系 Y 固定偏移(mm, +左)
 * @param cal_type        校准类型
 * @param out             输出: 相对修正量(机体系): pose.x/pose.y 位置(cm), pose.yaw 角度(deg)
 * @param out_yaw_err_deg 输出: 视觉 yaw 误差(°), 可为 NULL; 圆盘标定输出 0
 * @return 校准状态
 * @note 标定只输出修正量, 不再叠加当前位姿(位置修正由任务层交给底盘位置模式执行):
 *       位置修正 = (X 固定偏移 + 视觉 x 误差, Y 固定偏移 + 视觉 y 误差) cm,
 *       角度修正 = 视觉角度误差;
 *       任务层可选择不用该 yaw 误差转车身, 而把 out_yaw_err_deg 交给机械臂基座补偿
 *       (见 mission_config.h 的 MISSION_CAL_YAW_FIX_WITH_ARM)。
 */
bool RPI_Cal(const char *name,
             int offset_x_mm,
             int offset_y_mm,
             RPI_Cal_t cal_type,
             TargetPoint_t *out,
             float *out_yaw_err_deg) {
    if (out == NULL || name == NULL) return false;

    TargetPoint_t *target = Map_GetPointByName(name);
    if (target == NULL) return false;

    int16_t err_x, err_y;
    float err_angle_deg;
    if (!RPI_Calibrate(&err_x, &err_y, &err_angle_deg, cal_type)) return false;

    if (out_yaw_err_deg != NULL) {
        *out_yaw_err_deg = err_angle_deg;
    }

    /* 保留目标点的 id/name/motion 等字段, pose 改为输出相对修正量 */
    memcpy(out, target, sizeof(TargetPoint_t));

    float offset_x_cm = (float)offset_x_mm / 10.0f;
    float offset_y_cm = (float)offset_y_mm / 10.0f;

    out->pose.x = offset_x_cm + (float)err_x / 10.0f;
    out->pose.y = offset_y_cm + (float)err_y / 10.0f;
    out->pose.yaw = err_angle_deg;

    logInfo("rpi: cal '%s' err(%d, %d)mm off(%d, %d)mm -> delta(%.2f, %.2f)cm %.2fdeg",
            name,
            err_x,
            err_y,
            offset_x_mm,
            offset_y_mm,
            (double)out->pose.x,
            (double)out->pose.y,
            (double)out->pose.yaw);

    return true;
}

/* ==========================================================================
 * 5. 可选功能
 * ========================================================================== */

#if RPI_COMPLETENUM_ENABLED == 1

/**
 * @brief 树莓派获取已完成数量
 * @param picknum  输出: 已拾取数量
 * @param placenum 输出: 已放置数量
 * @return 获取状态
 * @note 应答帧: [0xFE, dir_1, dir_2, abs_1, abs_2], 无固定帧尾, 可能带前导字节
 */
bool RPI_SetCompleteNum(int *picknum, int *placenum) {
    if (picknum == NULL || placenum == NULL) {
        return false;
    }

    uint8_t rsp[RPI_RSP_LEN_LENIENT(RPI_PAYLOAD_LEN_COMPLETE)] = {0};
    uint16_t len =
        RPI_RequestFrame(RPI_CMD_COMPLETE_NUM, rsp, sizeof(rsp), RPI_PAYLOAD_LEN_COMPLETE, RPI_RSP_EXPECT_LENIENT);

    const uint8_t *pl = RPI_Frame_FindPayload(rsp, len, RPI_PAYLOAD_LEN_COMPLETE);
    if (pl == NULL) {
        return false;
    }

    *picknum = RPI_DecodeSignedZeroIsPos(pl[RPI_PAIR_IDX_DIR_1], pl[RPI_PAIR_IDX_ABS_1]);
    *placenum = RPI_DecodeSignedZeroIsPos(pl[RPI_PAIR_IDX_DIR_2], pl[RPI_PAIR_IDX_ABS_2]);

    return true;
}

#endif /* RPI_COMPLETENUM_ENABLED */

#if RPI_RADAR_AVAILABLE == 1

/**
 * @brief 树莓派激光雷达扫描单次轮询: 取一格并写入地图
 * @return 扫描状态
 * @note 雷达链路是"长会话"式轮询(每次 0x1F 取一格, 直到收到 0xEF 结束通知), 与单次命令不同:
 *       此路径不补发空闲命令 0x11 —— 那会在 Pi 扫描过程中反复把它拉回空闲态, 使 0xEF 永不到来;
 *       会话由 Pi 自己以 0xEF 结束并回到空闲, 无需 MCU 逐帧复位。
 */
RPI_ScanStatus_t RPI_SetGridVal(void) {
    uint8_t rsp[RPI_RSP_LEN_LENIENT(RPI_PAYLOAD_LEN_RADAR)] = {0};

    uint16_t len = RPI_Send_And_Receive(RPI_CMD_RADAR_SCAN, rsp, sizeof(rsp));

    if (len == 0) {
        return RPI_SCAN_ERROR;
    }

    /* 0xEF 为扫描完成通知(单字节), 用命令宏而非帧尾宏表达语义 */
    if (rsp[0] == RPI_CMD_RADAR_DONE) {
        logInfo("radar scan complete");
        return RPI_SCAN_DONE;
    }

    const uint8_t *pl = RPI_Frame_FindPayload(rsp, len, RPI_PAYLOAD_LEN_RADAR);
    if (pl == NULL) {
        return RPI_SCAN_ERROR;
    }

    int16_t grid_x = RPI_DecodeSignedZeroIsPos(pl[RPI_PAIR_IDX_DIR_1], pl[RPI_PAIR_IDX_ABS_1]);
    int16_t grid_y = RPI_DecodeSignedZeroIsPos(pl[RPI_PAIR_IDX_DIR_2], pl[RPI_PAIR_IDX_ABS_2]);

    /* 串口数据不可信: 越界格号经 set_grid_val 会算出越界索引, 野写 g_map.grid 之外的 BSS
       (g_map.grid 指向 planner 的 s_map_grid), 必须先做范围校验。
       注意: 这里按"树莓派直接给地图格号"处理, 不做 MAP_OFFSET 平移, 改协议时此处要同步。 */
    if (grid_x < 0 || grid_x >= g_map.grid_size || grid_y < 0 || grid_y >= g_map.grid_size) {
        logWarning("radar grid out of range: (%d, %d), size=%d", grid_x, grid_y, g_map.grid_size);
        /* 越界属无效数据, 计入失败以便上层重试, 避免"一直 ONGOING"导致扫描永不结束 */
        return RPI_SCAN_ERROR;
    }

    /* 动态障碍格: 与预定义静态障碍区分开(清障距离场只对动态格做保守几何近似) */
    set_grid_val(&g_map, grid_x, grid_y, MAP_CELL_DYNAMIC_OBSTACLE);

    return RPI_SCAN_ONGOING;
}

#endif /* RPI_RADAR_AVAILABLE */

/* ==========================================================================
 * 6. 调试: 接收脏数据统计
 * ========================================================================== */

/**
 * @brief 读取"非等待应答期间收到并丢弃"的数据统计(链路污染观察)
 * @param discarded_frames 输出: 丢弃事件计数(中断接收下约等于丢弃字节数), 可为 NULL
 * @param discarded_bytes  输出: 丢弃字节数, 可为 NULL
 * @note 正常运行时该值应基本不增长; 持续增长说明树莓派侧有非请求输出(重复帧/迟到应答),
 *       这些数据已被传输层丢弃, 不会进入命令解析。
 */
void RPI_GetRxStats(uint32_t *discarded_frames, uint32_t *discarded_bytes) {
    if (discarded_frames != NULL) {
        *discarded_frames = s_rx_discarded_frames;
    }
    if (discarded_bytes != NULL) {
        *discarded_bytes = s_rx_discarded_bytes;
    }
}

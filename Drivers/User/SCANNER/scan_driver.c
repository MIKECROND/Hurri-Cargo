/**
 * @file scan_driver.c
 * @author T-Car Team
 * @brief 扫描器驱动层（StreamBuffer 版）
 * 可调参数见 scan_driver.h。
 */

#include "scan_driver.h"
#include "log.h"
#include "usart.h"
#include <string.h>
#include "Events.h"
#include "task.h"

uint8_t rx5Buffer[USART5_RX_BUF_SIZE];

/* StreamBuffer 满导致的数据丢弃字节数：ISR 累加、任务侧限速打印（ISR 内禁止打印） */
static volatile uint32_t s_rx_dropped_bytes = 0;

/**
 * @brief 帧接收状态
 */
typedef struct {
    uint8_t buf[SCAN_FRAME_MAX_SIZE];
    uint16_t len;
    uint32_t last_time;
} Scan_Rx_t;

/**
 * @brief 暂存条码
 */
typedef struct {
    char data[SCAN_BARCODE_MAX_LEN + 1];
    uint32_t time; /**< 时间戳 */
    bool valid; /**< 是否有效 */
} Scan_Pending_t;

StreamBufferHandle_t Scan_Rx_Stream = NULL;
static bool is_init = false;
static Scan_Rx_t rx;
static Scan_Pending_t pending;

static bool scan_output_enabled = true;
static bool pending_output = false;

/**
 * @brief 从缓冲区提取可打印 ASCII 字符（过滤控制字符）
 * @param src  输入缓冲区指针
 * @param src_len 输入缓冲区长度
 * @param dst  输出缓冲区指针
 * @return 实际提取的字符数（包含 '\0'）
 */
static uint16_t extract_printable(const uint8_t *src, uint16_t src_len, uint8_t *dst) {
    uint16_t n = 0;
    for (uint16_t i = 0; i < src_len && n < SCAN_BARCODE_MAX_LEN; i++) {
        if (src[i] >= 0x20 && src[i] <= 0x7E) {
            dst[n++] = src[i];
        }
    }
    return n;
}

/**
 * @brief 清空接收缓冲区
 */
static void Scan_Clear_Buffer(void) {
    rx.len = 0;
    memset(rx.buf, 0, SCAN_FRAME_MAX_SIZE);
    rx.last_time = HAL_GetTick();
}

/**
 * @brief 解析条码并暂存（覆盖窗口内之前的结果）
 *
 * 帧格式: 0x02 | 0x00 | LEN | DATA(LEN 字节) | CR/LF
 *   [0] STX   = 0x02
 *   [1] 状态   = 0x00
 *   [2] 数据长度
 *   [3 .. 3+LEN-1] 条码数据
 */
static void parse_and_store(void) {
    if (rx.len == 0) return;

    uint8_t barcode[SCAN_BARCODE_MAX_LEN + 1] = {0};
    uint16_t barcode_len = 0;

    if (rx.len >= 4 && rx.buf[0] == 0x02 && rx.buf[1] == 0x00) {
        uint8_t data_len = rx.buf[2];
        if (data_len > 0 && (uint16_t)(3 + data_len) <= rx.len) {
            barcode_len = extract_printable(rx.buf + 3, data_len, barcode);
        }
    }

    if (barcode_len == 0) {
        barcode_len = extract_printable(rx.buf, rx.len, barcode);
    }

    if (barcode_len > 0) {
        barcode[barcode_len] = '\0';
        /* 临界区：pending 由本任务写、由 Mission 任务通过 Scan_GetLatestBarcode 读，
         * 原实现无保护，读取方可能拿到"写了一半"的条码字符串。复制只有 <64B，开销极小。 */
        taskENTER_CRITICAL();
        strncpy(pending.data, (char *)barcode, SCAN_BARCODE_MAX_LEN);
        pending.data[SCAN_BARCODE_MAX_LEN] = '\0';
        pending.time = HAL_GetTick();
        pending.valid = true;
        pending_output = false;
        taskEXIT_CRITICAL();
        /* 通知等待方(如总任务等待二维码)有新条码, 避免上层周期轮询 */
        osEventFlagsSet(System_StatusHandle, SCAN_EVT_BARCODE);
    }

    Scan_Clear_Buffer();
}

/**
 * @brief 分组窗口超时 → 输出暂存条码
 * @param now 当前时间戳（毫秒）
 */
static void flush_pending(uint32_t now) {
    if (pending.valid && !pending_output && (now - pending.time) >= SCAN_GROUP_WINDOW_MS) {
        if (scan_output_enabled) {
            logPrintln("[SCAN] %s", pending.data);
        }
        pending_output = true;
    }
}

/**
 * @brief 处理单个字节（状态机）
 * @param byte 接收到的字节
 * @param now  当前时间戳（毫秒）
 * *
 * CR(0x0D) / LF(0x0A) / 帧间超时 → 触发一次解析
 */
static void process_byte(uint8_t byte, uint32_t now) {
    rx.last_time = now;

    if (byte == 0x0D || byte == 0x0A) {
        if (rx.len > 0) {
            parse_and_store();
        }
        return; // 丢弃孤立的 CR/LF
    }

    if (rx.len < SCAN_FRAME_MAX_SIZE - 1) {
        rx.buf[rx.len++] = byte;
    } else {
        parse_and_store();
        rx.buf[0] = byte;
        rx.len = 1;
    }
}

void Scan_UART_RxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART5) {
        if (huart->RxEventType == HAL_UART_RXEVENT_TC || huart->RxEventType == HAL_UART_RXEVENT_IDLE) {
            if (Size > 0 && Scan_Rx_Stream != NULL) {
                BaseType_t xWoken = pdFALSE;
                size_t sent = xStreamBufferSendFromISR(Scan_Rx_Stream, rx5Buffer, Size, &xWoken);
                if (sent < (size_t)Size) {
                    s_rx_dropped_bytes += (uint32_t)((size_t)Size - sent);
                }
                portYIELD_FROM_ISR(xWoken);
            }

            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx5Buffer, USART5_RX_BUF_SIZE);
        }
    }
}

/**
 * @brief UART5 错误回调（ORE/帧错误/噪声/校验错）
 * @note  修复：原实现没有注册错误回调。HAL 在 DMA 接收模式下遇到任一错误都会
 *        UART_EndRxTransfer + Abort DMA 并调用 ErrorCallback；而 HAL 的默认弱定义
 *        是空函数，因此一次噪声/超时错误后 UART5 接收链路就永久停摆（扫码彻底失效，
 *        且没有任何日志提示）。此处清标志、恢复状态并重新启动 ReceiveToIdle_DMA。
 *        与 OPS(UART4)/Shell(USART1) 的错误回调保持一致。
 */
void Scan_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
        huart->RxState = HAL_UART_STATE_READY;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx5Buffer, USART5_RX_BUF_SIZE);
    }
}

/**
 * @brief 初始化扫描器驱动层
 */
bool Scan_Init(void) {
    MX_UART5_Init();

    memset(&rx, 0, sizeof(rx));
    memset(&pending, 0, sizeof(pending));

    /* 先创建 StreamBuffer、注册回调，最后才启动接收：
     * 原实现先启动 DMA 再创建 StreamBuffer，中间窗口收到的数据会因 Stream_Rx_Stream
     * 为 NULL 被静默丢弃（回调里虽有 NULL 判断，但数据已经丢了）。 */
    Scan_Rx_Stream = xStreamBufferCreate(SCAN_STREAM_BUF_SIZE, SCAN_STREAM_TRIGGER_LVL);
    if (Scan_Rx_Stream == NULL) {
        configASSERT(Scan_Rx_Stream != NULL);
        return false;
    }

    /* 注册接收事件回调函数 */
    if (HAL_UART_RegisterRxEventCallback(&huart5, Scan_UART_RxCallback) != HAL_OK) {
        Error_Handler();
    }

    /* 注册错误回调：否则一次 ORE/帧错误后接收链路永久失效（见 Scan_UART_ErrorCallback） */
    if (HAL_UART_RegisterCallback(&huart5, HAL_UART_ERROR_CB_ID, Scan_UART_ErrorCallback) != HAL_OK) {
        Error_Handler();
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx5Buffer, USART5_RX_BUF_SIZE);

    is_init = true;
    return is_init;
}

/**
 * @brief 启用扫描器输出
 */
void Scan_EnableOutput(void) {
    scan_output_enabled = true;
}

/**
 * @brief 禁用扫描器输出
 */
void Scan_DisableOutput(void) {
    scan_output_enabled = false;
}

/**
 * @brief 检查扫描器输出是否已启用
 * @return true 已启用 false 未启用
 */
bool Scan_IsOutputEnabled(void) {
    return scan_output_enabled;
}

/**
 * @brief 获取最新扫描到的条码
 * @param buf  存储条码的缓冲区
 * @param size 缓冲区大小（包含终止符 '\0'）
 * @return true 成功获取条码 false 无新条码
 */
bool Scan_GetLatestBarcode(uint8_t *buf, uint16_t size) {
    bool ok = false;

    if (buf == NULL || size == 0) return false; /* 防护: size==0 时 size-1 会下溢为极大值 */

    /* 与 ISR/解析侧共享 pending：临界区内完成"检查+复制+清有效标志"，避免撕裂 */
    taskENTER_CRITICAL();
    if (pending.valid) {
        strncpy((char *)buf, pending.data, size - 1);
        buf[size - 1] = '\0';
        pending.valid = false;
        ok = true;
    }
    taskEXIT_CRITICAL();

    return ok;
}

/**
 * @brief 扫描器接收任务
 *
 * 事件驱动：
 *   ISR 写入 StreamBuffer → xStreamBufferReceive 自动唤醒
 *   无数据时阻塞（带超时兜底处理不完整帧）
 */
void Scan_Get_Task(void *argument) {
    (void)argument;
    uint8_t buf[64];
    uint32_t now;

    osEventFlagsWait(System_StatusHandle, SYS_INIT_COMPLETE, osFlagsNoClear, osWaitForever);

    while (1) {
        /*
         * 阻塞读取：最多等 FRAME_TIMEOUT_MS
         *   - 有数据到达 → 立即返回，批量取出
         *   - 超时 → 返回 0，进入兜底逻辑
         */
        size_t n = xStreamBufferReceive(Scan_Rx_Stream, buf, sizeof(buf), pdMS_TO_TICKS(FRAME_TIMEOUT_MS));
        now = HAL_GetTick();

        if (n > 0) {
            for (size_t i = 0; i < n; i++) {
                process_byte(buf[i], now);
            }
        }

        /* StreamBuffer 溢出丢字节提示（ISR 只累加计数，打印放在任务里，且只在发生后才打） */
        if (s_rx_dropped_bytes != 0) {
            logWarning("SCAN rx stream overflow, %lu byte(s) dropped", (unsigned long)s_rx_dropped_bytes);
            s_rx_dropped_bytes = 0;
        }

        if (rx.len > 0 && (HAL_GetTick() - rx.last_time) >= FRAME_TIMEOUT_MS) {
            parse_and_store();
        }

        flush_pending(HAL_GetTick());
    }
}

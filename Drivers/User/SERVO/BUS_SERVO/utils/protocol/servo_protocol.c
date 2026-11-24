/**
 * @file servo_protocol.c
 * @brief 舵机模块协议实现
 */

#include "servo_protocol.h"
#include "servo_driver.h"
#include "log.h"
#include "stream_buffer.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Events.h"
#include "cmsis_os2.h"

Sync_ServoData servoSync[SERVO_MAX_COUNT]; // 同步控制舵机数量
ServoData servodata[SERVO_MAX_COUNT]; //舵机读取数据的数量

/* ==========================================================================
 * 事务串行化
 * 舵机总线是"请求-应答"式: 读类命令(查询/监控/参数读写/同步监控)从发请求到收应答
 * 之间不能被打断, 否则另一个任务的应答会被误当成本事务的数据(读错角度/长度不符)。
 * 这里用递归互斥把这类事务串行化, 使机械臂动作、电池电压监控、shell 调试等任务可共用总线。
 * ========================================================================== */
static osMutexId_t s_servo_txn_mutex = NULL;

/* 发送包构建缓冲(静态): Package_t 含 350B 负载, 不放任务栈上; 由事务锁保护 */
static Package_t s_tx_pkg;

void Servo_ProtocolInit(void) {
    if (s_servo_txn_mutex != NULL) {
        return;
    }
    const osMutexAttr_t attr = {
        .name = "ServoTxn",
        .attr_bits = osMutexRecursive, // 同步监控等实现内部会再嵌套调用事务接口
    };
    s_servo_txn_mutex = osMutexNew(&attr);
    if (s_servo_txn_mutex == NULL) {
        logError("servo: failed to create transaction mutex");
    }
}

static void servo_txn_lock(void) {
    if (s_servo_txn_mutex != NULL) {
        (void)osMutexAcquire(s_servo_txn_mutex, osWaitForever);
    }
}

static void servo_txn_unlock(void) {
    if (s_servo_txn_mutex != NULL) {
        (void)osMutexRelease(s_servo_txn_mutex);
    }
}

static SERVO_STATUS Servo_ReadByte(uint8_t *byte, uint32_t timeout_ms) {
    size_t n = xStreamBufferReceive(Servo_Rx_StreamHandle, byte, 1, pdMS_TO_TICKS(timeout_ms));
    return n == 1 ? SERVO_STATUS_SUCCESS : SERVO_STATUS_TIMEOUT;
}

uint16_t Servo_PackageToBytes(Package_t *pkg, uint8_t *buffer, uint16_t buffer_size) {
    uint16_t offset = 0;
    /* 扩展长度字段的判定必须与 Servo_CalcChecksum 完全一致 */
    uint8_t use_sync_len;
    uint16_t need;

    if ((pkg == NULL) || (buffer == NULL) || (pkg->size > SERVO_PACK_RESPONSE_MAX_SIZE)) {
        return 0;
    }

    use_sync_len = ((pkg->isSync != 0U) || (pkg->size > 255U)) ? 1U : 0U;
    need = (uint16_t)(5U + pkg->size + (use_sync_len ? 2U : 0U));
    if (need > buffer_size) {
        return 0;
    }

    /* 帧头（小端） */
    buffer[offset++] = pkg->header & 0xFF;
    buffer[offset++] = (pkg->header >> 8) & 0xFF;

    /* 命令 ID */
    buffer[offset++] = pkg->cmdId;

    /* 长度字段 */
    if (use_sync_len) {
        buffer[offset++] = 0xFF;
        buffer[offset++] = pkg->size & 0xFF;
        buffer[offset++] = (pkg->size >> 8) & 0xFF;
    } else {
        buffer[offset++] = pkg->size & 0xFF;
    }

    /* 内容 */
    for (uint16_t i = 0; i < pkg->size; i++) {
        buffer[offset++] = pkg->content[i];
    }

    /* checksum 在 Servo_SendPackage_Common 中已算好 */
    buffer[offset++] = pkg->checksum;

    return offset;
}

static uint8_t Servo_CalcChecksum(Package_t *pkg) {
    uint32_t sum = 0;
    uint8_t use_sync_len = ((pkg->isSync != 0U) || (pkg->size > 255U)) ? 1U : 0U;

    sum += (pkg->header >> 8) & 0xFF;
    sum += pkg->header & 0xFF;
    sum += pkg->cmdId;
    if (use_sync_len) {
        sum += 0xFF;
        sum += pkg->size & 0xFF;
        sum += (pkg->size >> 8) & 0xFF;
    } else {
        sum += pkg->size & 0xFF;
    }
    for (uint16_t i = 0; i < pkg->size; i++) {
        sum += pkg->content[i];
    }
    return (uint8_t)(sum % 256);
}

static SERVO_STATUS Servo_IsValidResponsePackage(Package_t *pkg) {
    if (pkg->header != SERVO_PACK_RESPONSE_HEADER) return SERVO_STATUS_WRONG_RESPONSE_HEADER;
    if (pkg->cmdId > SERVO_CMD_NUM) return SERVO_STATUS_UNKNOWN_CMD_ID;
    /* 与原库保持一致: 350 - 6, 防止 content 越界 */
    if (pkg->size > (SERVO_PACK_RESPONSE_MAX_SIZE - 6)) return SERVO_STATUS_SIZE_TOO_BIG;
    if (Servo_CalcChecksum(pkg) != pkg->checksum) return SERVO_STATUS_CHECKSUM_ERROR;
    return SERVO_STATUS_SUCCESS;
}

/**
 * @brief 丢弃接收流中残留的字节
 * @note  接收失败/超时后调用, 避免迟到的应答污染下一次事务
 */
static void Servo_FlushRx(void) {
    if (Servo_Rx_StreamHandle != NULL) {
        (void)xStreamBufferReset(Servo_Rx_StreamHandle);
    }
}

static SERVO_STATUS Servo_SendPackage_Common(uint8_t cmdId, uint16_t size, uint8_t *content, uint8_t isSync) {
    extern osMessageQueueId_t Servo_Tx_DataHandle;

    if (size > SERVO_PACK_RESPONSE_MAX_SIZE) {
        return SERVO_STATUS_SIZE_TOO_BIG;
    }
    if ((size > 0U) && (content == NULL)) {
        return SERVO_STATUS_FAIL;
    }

    /* 发送包在静态缓冲里构建: Package_t 含 350B content, 放在任务栈上会显著抬高峰值栈深,
     * 电池监控/机械臂动作等小栈任务容易栈溢出。构建+入队由事务锁串行化
     * (递归锁, 本身在事务内的调用也不会死锁)。 */
    servo_txn_lock();
    s_tx_pkg.header = SERVO_PACK_REQUEST_HEADER;
    s_tx_pkg.cmdId = cmdId;
    s_tx_pkg.size = size;
    s_tx_pkg.isSync = isSync;
    if (size > 0U) {
        memcpy(s_tx_pkg.content, content, size);
    }
    s_tx_pkg.checksum = Servo_CalcChecksum(&s_tx_pkg);

    SERVO_STATUS status;
    if (osMessageQueuePut(Servo_Tx_DataHandle, &s_tx_pkg, 0, 50) != osOK) {
        // 发送失败
        logError("Failed to send package to Servo");
        status = SERVO_STATUS_FAIL;
    } else {
        // 发送成功
        status = SERVO_STATUS_SUCCESS;
    }
    servo_txn_unlock();
    return status;
}

static SERVO_STATUS Servo_RecvPackage(Package_t *pkg) {
    uint8_t byte = 0;
    uint8_t header_byte1 = 0;
    uint32_t startTime;

    if (pkg == NULL) {
        return SERVO_STATUS_FAIL;
    }
    memset(pkg, 0, sizeof(Package_t));
    startTime = osKernelGetTickCount();

    while ((osKernelGetTickCount() - startTime) < SERVO_TIMEOUT_MS) {
        /* 1. 查找帧头: 内层循环也受总超时约束, 避免噪声导致长时间占用 */
        while ((osKernelGetTickCount() - startTime) < SERVO_TIMEOUT_MS &&
               Servo_ReadByte(&byte, 10) == SERVO_STATUS_SUCCESS) {
            if (header_byte1 == 0U && byte == 0x05U) {
                header_byte1 = byte;
            } else if (header_byte1 == 0x05U && byte == 0x1CU) {
                pkg->header = SERVO_PACK_RESPONSE_HEADER;
                /* 匹配成功后必须复位, 否则残留的 0x05 会漏掉下一帧的帧头 */
                header_byte1 = 0U;
                break;
            } else {
                header_byte1 = 0U;
            }
        }

        if (pkg->header != SERVO_PACK_RESPONSE_HEADER) {
            continue;
        }

        /* 2. 接收 cmdId */
        if (Servo_ReadByte(&pkg->cmdId, 10) != SERVO_STATUS_SUCCESS) {
            Servo_FlushRx();
            return SERVO_STATUS_TIMEOUT;
        }

        /* 3. 接收 size（判断同步模式） */
        if (Servo_ReadByte(&byte, 10) != SERVO_STATUS_SUCCESS) {
            Servo_FlushRx();
            return SERVO_STATUS_TIMEOUT;
        }

        if (byte == 0xFFU) {
            /* 同步模式：size 占 2 字节 */
            uint8_t sizeLow;
            uint8_t sizeHigh;

            pkg->isSync = 1U;
            if (Servo_ReadByte(&sizeLow, 10) != SERVO_STATUS_SUCCESS) {
                Servo_FlushRx();
                return SERVO_STATUS_TIMEOUT;
            }
            if (Servo_ReadByte(&sizeHigh, 10) != SERVO_STATUS_SUCCESS) {
                Servo_FlushRx();
                return SERVO_STATUS_TIMEOUT;
            }
            pkg->size = (uint16_t)(sizeLow | (sizeHigh << 8));
        } else {
            /* 普通模式：size 占 1 字节 */
            pkg->isSync = 0U;
            pkg->size = byte;
        }

        /* 4. 检查 size 有效性（与原库一致: 允许 0 长度, 超过上限直接报错） */
        if (pkg->size > SERVO_PACK_RESPONSE_MAX_SIZE) {
            Servo_FlushRx();
            return SERVO_STATUS_SIZE_TOO_BIG;
        }

        /* 5. content 一次性批量读取 */
        if (pkg->size > 0U) {
            size_t received = xStreamBufferReceive(Servo_Rx_StreamHandle, pkg->content, pkg->size, pdMS_TO_TICKS(10));
            if (received != pkg->size) {
                Servo_FlushRx();
                return SERVO_STATUS_TIMEOUT;
            }
        }

        /* 6. 接收 checksum */
        if (Servo_ReadByte(&pkg->checksum, 10) != SERVO_STATUS_SUCCESS) {
            Servo_FlushRx();
            return SERVO_STATUS_TIMEOUT;
        }

        /* 7. 验证包有效性 */
        SERVO_STATUS status = Servo_IsValidResponsePackage(pkg);
        if (status != SERVO_STATUS_SUCCESS) {
            Servo_FlushRx();
        }
        return status;
    }

    Servo_FlushRx();
    return SERVO_STATUS_TIMEOUT;
}

static SERVO_STATUS Servo_Sync_RecvPackage(Package_t *pkg) {
    uint8_t byte;
    uint8_t header = 0;
    uint8_t bIdx = 0;
    uint32_t startTime;

    if (pkg == NULL) {
        return SERVO_STATUS_FAIL;
    }
    // 初始化包状态
    pkg->status = 0;
    startTime = osKernelGetTickCount();

    // 超时等待（100ms）
    while ((osKernelGetTickCount() - startTime) < SERVO_TIMEOUT_MS) {
        // 从流缓冲读取一个字节: 第3个参数是目标缓冲区容量, 必须是 1
        if (xStreamBufferReceive(Servo_Rx_StreamHandle, &byte, 1, pdMS_TO_TICKS(10)) != 1U) {
            continue;
        }

        // 帧头同步检测：严格匹配 0x05 0x1C
        if ((pkg->status & SERVO_RECV_FLAG_HEADER) == 0) {
            if (header == 0 && byte == 0x05) {
                header = byte; // 帧头第1字节
            } else if (header == 0x05 && byte == 0x1C) {
                pkg->header = 0x1C05; // 帧头检测成功
                pkg->status |= SERVO_RECV_FLAG_HEADER;
                header = 0; // 重置帧头
            } else {
                header = 0; // 帧头检测失败，重置
            }
            continue;
        }

        // 解析 cmdId
        if ((pkg->status & SERVO_RECV_FLAG_CMD_ID) == 0) {
            pkg->cmdId = byte;
            if (pkg->cmdId != 22) { // 检查cmdId必须为22
                Servo_FlushRx();
                return SERVO_STATUS_UNKNOWN_CMD_ID;
            }
            pkg->status |= SERVO_RECV_FLAG_CMD_ID;
            continue;
        }

        // 解析 size
        if ((pkg->status & SERVO_RECV_FLAG_SIZE) == 0) {
            pkg->size = byte;
            if (pkg->size != 16) { // 每组数据16字节
                Servo_FlushRx();
                return SERVO_STATUS_SIZE_TOO_BIG;
            }
            pkg->status |= SERVO_RECV_FLAG_SIZE;
            continue;
        }

        // 读取 content
        if ((pkg->status & SERVO_RECV_FLAG_CONTENT) == 0) {
            pkg->content[bIdx++] = byte;
            if (bIdx >= pkg->size) {
                pkg->status |= SERVO_RECV_FLAG_CONTENT;
            }
            continue;
        }

        // 校验和
        if ((pkg->status & SERVO_RECV_FLAG_CHECKSUM) == 0) {
            pkg->checksum = byte;
            if (Servo_CalcChecksum(pkg) != pkg->checksum) {
                Servo_FlushRx();
                return SERVO_STATUS_CHECKSUM_ERROR;
            }
            return SERVO_STATUS_SUCCESS;
        }
    }

    Servo_FlushRx();
    return SERVO_STATUS_TIMEOUT;
}

static SERVO_STATUS servo_ping_impl(uint8_t servo_id) {
    SERVO_STATUS statusCode; // 状态码
    uint8_t ehcoServoId; // PING得到的舵机ID
    // printf("[PING]Send Ping Package\r\n");
    // 发送请求包
    Servo_SendPackage_Common(SERVO_CMD_PING, 1, &servo_id, 0);
    // 接收返回的Ping
    Package_t pkg = {0};
    statusCode = Servo_RecvPackage(&pkg);
    if (statusCode == SERVO_STATUS_SUCCESS) {
        // content 至少要有舵机 ID 才能继续解析
        if (pkg.size < 1U) {
            return SERVO_STATUS_SIZE_TOO_BIG;
        }
        // 进一步检查ID号是否匹配
        ehcoServoId = (uint8_t)pkg.content[0];
        if (ehcoServoId != servo_id) {
            // 反馈得到的舵机ID号不匹配
            return SERVO_STATUS_ID_NOT_MATCH;
        }
    }
    return statusCode;
}

static SERVO_STATUS servo_reset_user_data_impl(uint8_t servo_id) {
    const uint8_t size = 1;
    SERVO_STATUS statusCode;
    // 发送请求包
    Servo_SendPackage_Common(SERVO_CMD_RESET_USER_DATA, size, &servo_id, 0);
    // 接收重置结果
    Package_t pkg;
    statusCode = Servo_RecvPackage(&pkg);
    if (statusCode == SERVO_STATUS_SUCCESS) {
        // 成功的接收到反馈数据
        // content = servo_id + result, 长度不足 2 时不能解析
        if (pkg.size < 2U) {
            return SERVO_STATUS_SIZE_TOO_BIG;
        }
        // 读取反馈数据中的result
        uint8_t result = (uint8_t)pkg.content[1];
        if (result == 1) {
            return SERVO_STATUS_SUCCESS;
        } else {
            return SERVO_STATUS_FAIL;
        }
    }
    return statusCode;
}

static SERVO_STATUS servo_read_data_impl(uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t *size) {
    SERVO_STATUS statusCode;
    uint8_t buffer[2] = {servo_id, address};
    Servo_SendPackage_Common(SERVO_CMD_READ_DATA, 2, buffer, 0);
    Package_t pkg;
    statusCode = Servo_RecvPackage(&pkg);
    if (statusCode == SERVO_STATUS_SUCCESS) {
        /* content = servo_id + address + data..., 长度不足 2 或超过调用方可接收上限时不能解析 */
        if ((pkg.size < 2U) || ((uint16_t)(pkg.size - 2U) > SERVO_READ_DATA_MAX_SIZE)) {
            return SERVO_STATUS_SIZE_TOO_BIG;
        }
        // 读取数据
        // 读取数据是多少个位
        *size = pkg.size - 2; // content的长度减去servo_id跟address的长度
        // 数据拷贝
        for (int i = 0; i < *size; i++) {
            value[i] = pkg.content[i + 2];
        }
    }
    return statusCode;
}

static SERVO_STATUS servo_write_data_impl(uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t size) {
    SERVO_STATUS statusCode;
    // 构造content
    uint8_t buffer[size + 2]; // 舵机ID + 地址位Address + 数据byte数
    buffer[0] = servo_id;
    buffer[1] = address;
    // 拷贝数据
    for (int i = 0; i < size; i++) {
        buffer[i + 2] = value[i];
    }
    // 发送请求数据
    Servo_SendPackage_Common(SERVO_CMD_WRITE_DATA, size + 2, buffer, 0);
    // 接收返回信息
    Package_t pkg;
    statusCode = Servo_RecvPackage(&pkg);
    if (statusCode == SERVO_STATUS_SUCCESS) {
        /* content = servo_id + address + result, 长度不足 3 时不能解析 */
        if (pkg.size < 3U) {
            return SERVO_STATUS_SIZE_TOO_BIG;
        }
        uint8_t result = pkg.content[2];
        if (result == 1) {
            statusCode = SERVO_STATUS_SUCCESS;
        } else {
            statusCode = SERVO_STATUS_FAIL;
        }
    }
    return statusCode;
}

SERVO_STATUS Servo_SetServoAngle(uint8_t servo_id, float angle, uint16_t interval, uint16_t power) {
    const uint8_t size = 7;

    // 数值约束
    if (angle > 180.0f) {
        angle = 180.0f;
    } else if (angle < -180.0f) {
        angle = -180.0f;
    }

    int16_t scaledAngle = (int16_t)(10 * angle);

    uint8_t content[size];
    content[0] = servo_id; // 1字节
    content[1] = scaledAngle & 0xFF; // 角度低字节
    content[2] = (scaledAngle >> 8) & 0xFF; // 角度高字节
    content[3] = interval & 0xFF; // interval低字节
    content[4] = (interval >> 8) & 0xFF; // interval高字节
    content[5] = power & 0xFF; // power低字节
    content[6] = (power >> 8) & 0xFF; // power高字节

    return Servo_SendPackage_Common(SERVO_CMD_ROTATE, size, content, 0);
}

SERVO_STATUS Servo_SetServoAngleByInterval(
    uint8_t servo_id, float angle, uint16_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power) {
    const uint8_t size = 11;

    // 数值约束
    if (angle > 180.0f) {
        angle = 180.0f;
    } else if (angle < -180.0f) {
        angle = -180.0f;
    }
    if (t_acc < 20) {
        t_acc = 20;
    }
    if (t_dec < 20) {
        t_dec = 20;
    }
    int16_t scaledAngle = (int16_t)(10 * angle);

    // 协议打包
    uint8_t content[size];
    content[0] = servo_id; // 1字节：舵机ID
    content[1] = scaledAngle & 0xFF; // 角度低字节
    content[2] = (scaledAngle >> 8) & 0xFF; // 角度高字节
    content[3] = interval & 0xFF; // interval低字节
    content[4] = (interval >> 8) & 0xFF; // interval高字节
    content[5] = t_acc & 0xFF; // t_acc低字节
    content[6] = (t_acc >> 8) & 0xFF; // t_acc高字节
    content[7] = t_dec & 0xFF; // t_dec低字节
    content[8] = (t_dec >> 8) & 0xFF; // t_dec高字节
    content[9] = power & 0xFF; // power低字节
    content[10] = (power >> 8) & 0xFF; // power高字节

    // 发送请求包
    return Servo_SendPackage_Common(SERVO_CMD_SET_ANGLE_BY_INTERVAL, size, content, 0);
}

SERVO_STATUS Servo_SetServoAngleByVelocity(
    uint8_t servo_id, float angle, float velocity, uint16_t t_acc, uint16_t t_dec, uint16_t power) {
    // 创建环形缓冲队列
    const uint8_t size = 11;

    // 数值约束
    if (angle > 180.0f) {
        angle = 180.0f;
    } else if (angle < -180.0f) {
        angle = -180.0f;
    }
    if (velocity < 1.0f) {
        velocity = 1.0f;
    } else if (velocity > 750.0f) {
        velocity = 750.0f;
    }
    if (t_acc < 20) {
        t_acc = 20;
    }
    if (t_dec < 20) {
        t_dec = 20;
    }
    int16_t scaledAngle = (int16_t)(10 * angle);
    uint16_t scaledVelocity = (uint16_t)(10 * velocity);

    // 协议打包
    uint8_t content[size];
    content[0] = servo_id; // 1字节：舵机ID
    content[1] = scaledAngle & 0xFF; // 角度低字节
    content[2] = (scaledAngle >> 8) & 0xFF; // 角度高字节
    content[3] = scaledVelocity & 0xFF; // velocity低字节
    content[4] = (scaledVelocity >> 8) & 0xFF; // velocity高字节
    content[5] = t_acc & 0xFF; // t_acc低字节
    content[6] = (t_acc >> 8) & 0xFF; // t_acc高字节
    content[7] = t_dec & 0xFF; // t_dec低字节
    content[8] = (t_dec >> 8) & 0xFF; // t_dec高字节
    content[9] = power & 0xFF; // power低字节
    content[10] = (power >> 8) & 0xFF; // power高字节

    // 发送请求包
    return Servo_SendPackage_Common(SERVO_CMD_SET_ANGLE_BY_VELOCITY, size, content, 0);
}

static SERVO_STATUS servo_query_angle_impl(uint8_t servo_id, float *angle) {
    const uint8_t size = 1; // 请求包content的长度
    uint8_t ehcoServoId;
    int16_t echoAngle;

    // 发送舵机角度请求包
    Servo_SendPackage_Common(SERVO_CMD_READ_ANGLE, size, &servo_id, 0);
    // 接收返回的Ping
    Package_t pkg;
    SERVO_STATUS statusCode = Servo_RecvPackage(&pkg);
    if (statusCode == SERVO_STATUS_SUCCESS) {
        /* content = servo_id + int16 angle */
        if (pkg.size < 3U) {
            return SERVO_STATUS_SIZE_TOO_BIG;
        }
        // 成功的获取到舵机角度回读数据
        ehcoServoId = (uint8_t)pkg.content[0];
        // 检测舵机ID是否匹配
        if (ehcoServoId != servo_id) {
            // 反馈得到的舵机ID号不匹配
            return SERVO_STATUS_ID_NOT_MATCH;
        }

        // 提取舵机角度
        echoAngle = (int16_t)(pkg.content[1] | (pkg.content[2] << 8));
        *angle = (float)echoAngle / 10.0f;
    }
    return statusCode;
}

SERVO_STATUS Servo_SetServoAngleMTurn(uint8_t servo_id, float angle, uint32_t interval, uint16_t power) {
    const uint8_t size = 11;

    // 数值约束
    if (angle > 368640.0f) {
        angle = 368640.0f;
    } else if (angle < -368640.0f) {
        angle = -368640.0f;
    }
    if (interval > 4096000) {
        interval = 4096000;
    }

    int32_t scaledAngle = (int32_t)(10 * angle);

    // 协议打包
    uint8_t content[size];
    content[0] = servo_id;
    content[1] = scaledAngle & 0xFF;
    content[2] = (scaledAngle >> 8) & 0xFF;
    content[3] = (scaledAngle >> 16) & 0xFF;
    content[4] = (scaledAngle >> 24) & 0xFF;
    content[5] = interval & 0xFF;
    content[6] = (interval >> 8) & 0xFF;
    content[7] = (interval >> 16) & 0xFF;
    content[8] = (interval >> 24) & 0xFF;
    content[9] = power & 0xFF;
    content[10] = (power >> 8) & 0xFF;

    // 发送请求包
    return Servo_SendPackage_Common(SERVO_CMD_SET_ANGLE_MTURN, size, content, 0);
}

SERVO_STATUS Servo_SetServoAngleMTurnByInterval(
    uint8_t servo_id, float angle, uint32_t interval, uint16_t t_acc, uint16_t t_dec, uint16_t power) {
    const uint8_t size = 15;

    // 数值约束
    if (angle > 368640.0f) {
        angle = 368640.0f;
    } else if (angle < -368640.0f) {
        angle = -368640.0f;
    }
    if (interval > 4096000) {
        interval = 4096000;
    }
    if (t_acc < 20) {
        t_acc = 20;
    }
    if (t_dec < 20) {
        t_dec = 20;
    }

    int32_t scaledAngle = (int32_t)(10 * angle);

    // 协议打包
    uint8_t content[size];
    content[0] = servo_id; // 1字节：舵机ID
    content[1] = scaledAngle & 0xFF; // 角度字节0
    content[2] = (scaledAngle >> 8) & 0xFF; // 角度字节1
    content[3] = (scaledAngle >> 16) & 0xFF; // 角度字节2
    content[4] = (scaledAngle >> 24) & 0xFF; // 角度字节3
    content[5] = interval & 0xFF; // interval字节0
    content[6] = (interval >> 8) & 0xFF; // interval字节1
    content[7] = (interval >> 16) & 0xFF; // interval字节2
    content[8] = (interval >> 24) & 0xFF; // interval字节3
    content[9] = t_acc & 0xFF; // t_acc低字节
    content[10] = (t_acc >> 8) & 0xFF; // t_acc高字节
    content[11] = t_dec & 0xFF; // t_dec低字节
    content[12] = (t_dec >> 8) & 0xFF; // t_dec高字节
    content[13] = power & 0xFF; // power低字节
    content[14] = (power >> 8) & 0xFF; // power高字节

    // 发送请求包
    return Servo_SendPackage_Common(SERVO_CMD_SET_ANGLE_MTURN_BY_INTERVAL, size, content, 0);
}

SERVO_STATUS Servo_SetServoAngleMTurnByVelocity(
    uint8_t servo_id, float angle, float velocity, uint16_t t_acc, uint16_t t_dec, uint16_t power) {
    // 创建环形缓冲队列
    const uint8_t size = 13;

    // 数值约束
    if (angle > 368640.0f) {
        angle = 368640.0f;
    } else if (angle < -368640.0f) {
        angle = -368640.0f;
    }
    if (velocity < 1.0f) {
        velocity = 1.0f;
    } else if (velocity > 750.0f) {
        velocity = 750.0f;
    }
    if (t_acc < 20) {
        t_acc = 20;
    }
    if (t_dec < 20) {
        t_dec = 20;
    }
    int32_t scaledAngle = (int32_t)(10 * angle);
    int16_t scaledVelocity = (int16_t)(10 * velocity);

    // 协议打包
    uint8_t content[size];
    content[0] = servo_id; // 1字节：舵机ID
    content[1] = scaledAngle & 0xFF; // 角度字节0
    content[2] = (scaledAngle >> 8) & 0xFF; // 角度字节1
    content[3] = (scaledAngle >> 16) & 0xFF; // 角度字节2
    content[4] = (scaledAngle >> 24) & 0xFF; // 角度字节3
    content[5] = scaledVelocity & 0xFF; // 速度低字节
    content[6] = (scaledVelocity >> 8) & 0xFF; // 速度高字节
    content[7] = t_acc & 0xFF; // t_acc低字节
    content[8] = (t_acc >> 8) & 0xFF; // t_acc高字节
    content[9] = t_dec & 0xFF; // t_dec低字节
    content[10] = (t_dec >> 8) & 0xFF; // t_dec高字节
    content[11] = power & 0xFF; // power低字节
    content[12] = (power >> 8) & 0xFF; // power高字节

    // 发送请求包
    return Servo_SendPackage_Common(SERVO_CMD_SET_ANGLE_MTURN_BY_VELOCITY, size, content, 0);
}

static SERVO_STATUS servo_query_angle_mturn_impl(uint8_t servo_id, float *angle) {
    // 创建环形缓冲队列
    const uint8_t size = 1; // 请求包content的长度
    uint8_t ehcoServoId;
    int32_t echoAngle;

    // 发送舵机角度请求包
    Servo_SendPackage_Common(SERVO_CMD_QUERY_ANGLE_MTURN, size, &servo_id, 0);
    // 接收返回的Ping
    Package_t pkg;
    SERVO_STATUS statusCode = Servo_RecvPackage(&pkg);
    if (statusCode == SERVO_STATUS_SUCCESS) {
        /* content = servo_id + int32 angle */
        if (pkg.size < 5U) {
            return SERVO_STATUS_SIZE_TOO_BIG;
        }
        // 成功的获取到舵机角度回读数据
        ehcoServoId = (uint8_t)pkg.content[0];
        // 检测舵机ID是否匹配
        if (ehcoServoId != servo_id) {
            // 反馈得到的舵机ID号不匹配
            return SERVO_STATUS_ID_NOT_MATCH;
        }

        // 提取舵机角度
        echoAngle = (int32_t)(pkg.content[1] | (pkg.content[2] << 8) | (pkg.content[3] << 16) | (pkg.content[4] << 24));
        *angle = (float)echoAngle / 10.0f;
    }
    return statusCode;
}

SERVO_STATUS Servo_DampingMode(uint8_t servo_id, uint16_t power) {
    const uint8_t size = 3; // 请求包content的长度

    // 构造content
    uint8_t content[size];
    content[0] = servo_id;
    content[1] = power & 0xFF;
    content[2] = (power >> 8) & 0xFF;
    // 发送请求包
    return Servo_SendPackage_Common(SERVO_CMD_DAMPING, size, content, 0);
}

SERVO_STATUS Servo_ResetServoMTurnAngle(uint8_t servo_id) {
    /* 原库对应指令为 FSUS_CMD_RESERT_SERVO_ANGLE_MTURN(17):
	   content 只有舵机 ID, 且不等待应答 */
    return Servo_SendPackage_Common(SERVO_CMD_RESET_ANGLE_MTURN, 1, &servo_id, 0);
}

SERVO_STATUS Servo_SetOriginPoint(uint8_t servo_id) {
    /* 原库此处以 size=2 传入 1 字节变量(越界读), 实际协议 content 只有舵机 ID;
	   原库不等待应答, 这里保持一致(是否回应取决于舵机的反馈开关) */
    return Servo_SendPackage_Common(SERVO_CMD_SET_ORIGIN_POINT, 1, &servo_id, 0);
}

SERVO_STATUS Servo_BeginAsync(void) {
    const uint8_t size = 0; // 请求包content的长度
    // 构造content
    uint8_t *content = NULL;
    // 发送请求包
    return Servo_SendPackage_Common(SERVO_CMD_BEGIN_ASYNC, size, content, 0);
}

SERVO_STATUS Servo_EndAsync(uint8_t mode) {
    /*mode
	0:执行存储的命令
	1:取消存储的命令
*/
    const uint8_t size = 1; // 请求包content的长度
    uint8_t content[size];
    content[0] = mode;
    // 发送请求包
    return Servo_SendPackage_Common(SERVO_CMD_END_ASYNC, size, content, 0);
}

static SERVO_STATUS servo_monitor_impl(uint8_t servo_id, ServoData servodata[]) {
    // 请求包content: 只有舵机 ID
    const uint8_t size = 1;

    uint8_t content[size];
    content[0] = servo_id;

    Servo_SendPackage_Common(SERVO_CMD_SET_SERVO_ReadData, (uint8_t)size, content, 0);

    /* 接收包放静态缓冲: 电池电压监控等小栈任务会周期调用本接口,
     * Package_t(~358B)放栈上会抬高峰值栈深; 事务锁已保证同一时刻只有一个监控事务。 */
    static Package_t pkg;
    SERVO_STATUS status = Servo_RecvPackage(&pkg);

    if (status != SERVO_STATUS_SUCCESS) {
        return status; // 如果接收失败，返回错误状态
    }

    /* 监控应答固定为 16 字节 content, 长度不符时不能按固定下标解析 */
    if ((pkg.cmdId != SERVO_CMD_SET_SERVO_ReadData) || (pkg.size != 16U)) {
        return SERVO_STATUS_SIZE_TOO_BIG;
    }

    // 解析当前数据包内容
    servodata[0].id = pkg.content[0];
    servodata[0].voltage = (int16_t)((pkg.content[2] << 8) | pkg.content[1]);
    servodata[0].current = (int16_t)((pkg.content[4] << 8) | pkg.content[3]);
    servodata[0].power = (int16_t)((pkg.content[6] << 8) | pkg.content[5]);

    /* 温度: ADC 值需在 (0,4096) 内才能做 Steinhart 换算, 与原库同步版本一致加保护 */
    {
        uint16_t adc_temp = (uint16_t)((pkg.content[8] << 8) | pkg.content[7]);
        if ((adc_temp > 0U) && (adc_temp < 4096U)) {
            double temp = (double)adc_temp;
            double kelvin = 1.0 / (log(temp / (4096.0 - temp)) / 3435.0 + 1.0 / (273.15 + 25.0));
            servodata[0].temperature = (int16_t)(kelvin - 273.15);
        } else {
            servodata[0].temperature = 0;
        }
    }

    servodata[0].status = pkg.content[9];

    /* 先按 uint32 拼接再转 int32, 避免 pkg.content[13] << 24 的有符号左移溢出(UB) */
    {
        int32_t raw_angle = (int32_t)(((uint32_t)pkg.content[13] << 24) | ((uint32_t)pkg.content[12] << 16) |
                                      ((uint32_t)pkg.content[11] << 8) | ((uint32_t)pkg.content[10]));
        servodata[0].angle = (float)raw_angle / 10.0f;
    }
    servodata[0].circle_count = (int16_t)((pkg.content[15] << 8) | pkg.content[14]);

    return SERVO_STATUS_SUCCESS;
}

//mode 指令停止形式
//0-停止后卸力(失锁)
//1-停止后保持锁力
//2-停止后进入阻尼状态
SERVO_STATUS Servo_StopOnControlMode(uint8_t servo_id, uint8_t mode, uint16_t power) {
    // 创建环形缓冲队列
    const uint8_t size = 4;
    uint8_t content[size];
    content[0] = servo_id;
    content[1] = mode | 0x10;
    content[2] = power & 0xFF;
    content[3] = (power >> 8) & 0xFF;

    return Servo_SendPackage_Common(SERVO_CMD_CONTROL_MODE_STOP, (uint8_t)size, content, 0);
}

static SERVO_STATUS servo_sync_command_impl(uint8_t servo_count, uint8_t servo_mode, Sync_ServoData servoSync[]) {
    uint16_t size;
    uint8_t *content = NULL;
    SERVO_STATUS status;

    if ((servoSync == NULL) || (servo_count == 0U) || (servo_count > SERVO_MAX_COUNT)) {
        return SERVO_STATUS_FAIL;
    }

    switch (servo_mode) {
    case MODE_SET_SERVO_ANGLE:
        /* 同步命令设置舵机的角度(单圈模式）*/
        size = 3 + servo_count * 7;
        content = (uint8_t *)pvPortMalloc(size);
        if (content == NULL) return SERVO_STATUS_FAIL;

        content[0] = 8; // 指令标识
        content[1] = 7; // 参数长度
        content[2] = servo_count; // 舵机数量

        for (int i = 0; i < servo_count; i++) {
            int16_t angle;
            if (servoSync[i].angle > 180.0f) {
                angle = (int16_t)(10 * 180.0f);
            } else if (servoSync[i].angle < -180.0f) {
                angle = (int16_t)(10 * -180.0f);
            } else {
                angle = (int16_t)(10 * servoSync[i].angle);
            }
            uint16_t interval = servoSync[i].interval_single;
            uint16_t power = servoSync[i].power;

            uint8_t *p = content + 3 + i * 7;
            p[0] = servoSync[i].id;
            p[1] = angle & 0xFF;
            p[2] = (angle >> 8) & 0xFF;
            p[3] = interval & 0xFF;
            p[4] = (interval >> 8) & 0xFF;
            p[5] = power & 0xFF;
            p[6] = (power >> 8) & 0xFF;
        }
        break;

    case MODE_SET_SERVO_ANGLE_BY_INTERVAL:
        /* 同步命令设置舵机的角度(单圈模式，指定周期) */
        size = 3 + servo_count * 11;
        content = (uint8_t *)pvPortMalloc(size);
        if (content == NULL) return SERVO_STATUS_FAIL;

        content[0] = 11;
        content[1] = 11;
        content[2] = servo_count;

        for (int i = 0; i < servo_count; i++) {
            int16_t angle;
            if (servoSync[i].angle > 180.0f) {
                angle = (int16_t)(10 * 180.0f);
            } else if (servoSync[i].angle < -180.0f) {
                angle = (int16_t)(10 * -180.0f);
            } else {
                angle = (int16_t)(10 * servoSync[i].angle);
            }

            uint8_t *p = content + 3 + i * 11;
            p[0] = servoSync[i].id;
            p[1] = angle & 0xFF;
            p[2] = (angle >> 8) & 0xFF;
            p[3] = servoSync[i].interval_single & 0xFF;
            p[4] = (servoSync[i].interval_single >> 8) & 0xFF;
            p[5] = servoSync[i].t_acc & 0xFF;
            p[6] = (servoSync[i].t_acc >> 8) & 0xFF;
            p[7] = servoSync[i].t_dec & 0xFF;
            p[8] = (servoSync[i].t_dec >> 8) & 0xFF;
            p[9] = servoSync[i].power & 0xFF;
            p[10] = (servoSync[i].power >> 8) & 0xFF;
        }
        break;

    case MODE_SET_SERVO_ANGLE_BY_VELOCITY:
        /* 同步命令设置舵机的角度(单圈模式，指定转速) */
        size = 3 + servo_count * 11;
        content = (uint8_t *)pvPortMalloc(size);
        if (content == NULL) return SERVO_STATUS_FAIL;

        content[0] = 12;
        content[1] = 11;
        content[2] = servo_count;

        for (int i = 0; i < servo_count; i++) {
            int16_t angle;
            if (servoSync[i].angle > 180.0f) {
                angle = (int16_t)(10 * 180.0f);
            } else if (servoSync[i].angle < -180.0f) {
                angle = (int16_t)(10 * -180.0f);
            } else {
                angle = (int16_t)(10 * servoSync[i].angle);
            }
            uint16_t velocity = (uint16_t)(10.0f * servoSync[i].velocity);

            uint8_t *p = content + 3 + i * 11;
            p[0] = servoSync[i].id;
            p[1] = angle & 0xFF;
            p[2] = (angle >> 8) & 0xFF;
            p[3] = velocity & 0xFF;
            p[4] = (velocity >> 8) & 0xFF;
            p[5] = servoSync[i].t_acc & 0xFF;
            p[6] = (servoSync[i].t_acc >> 8) & 0xFF;
            p[7] = servoSync[i].t_dec & 0xFF;
            p[8] = (servoSync[i].t_dec >> 8) & 0xFF;
            p[9] = servoSync[i].power & 0xFF;
            p[10] = (servoSync[i].power >> 8) & 0xFF;
        }
        break;

    case MODE_SET_SERVO_ANGLE_MTURN:
        /* 同步命令设置舵机的角度(多圈模式) */
        size = 3 + servo_count * 11;
        content = (uint8_t *)pvPortMalloc(size);
        if (content == NULL) return SERVO_STATUS_FAIL;

        content[0] = 13;
        content[1] = 11;
        content[2] = servo_count;

        for (int i = 0; i < servo_count; i++) {
            int32_t angle;
            if (servoSync[i].angle > 368640.0f) {
                angle = (int32_t)(10 * 368640.0f);
            } else if (servoSync[i].angle < -368640.0f) {
                angle = (int32_t)(10 * -368640.0f);
            } else {
                angle = (int32_t)(10 * servoSync[i].angle);
            }

            uint8_t *p = content + 3 + i * 11;
            p[0] = servoSync[i].id;
            p[1] = angle & 0xFF;
            p[2] = (angle >> 8) & 0xFF;
            p[3] = (angle >> 16) & 0xFF;
            p[4] = (angle >> 24) & 0xFF;
            p[5] = servoSync[i].interval_multi & 0xFF;
            p[6] = (servoSync[i].interval_multi >> 8) & 0xFF;
            p[7] = (servoSync[i].interval_multi >> 16) & 0xFF;
            p[8] = (servoSync[i].interval_multi >> 24) & 0xFF;
            p[9] = servoSync[i].power & 0xFF;
            p[10] = (servoSync[i].power >> 8) & 0xFF;
        }
        break;

    case MODE_SET_SERVO_ANGLE_MTURN_BY_INTERVAL:
        /* 同步命令设置舵机的角度(多圈模式, 指定周期) */
        size = 3 + servo_count * 15;
        content = (uint8_t *)pvPortMalloc(size);
        if (content == NULL) return SERVO_STATUS_FAIL;

        content[0] = 14;
        content[1] = 15;
        content[2] = servo_count;

        for (int i = 0; i < servo_count; i++) {
            int32_t angle;
            if (servoSync[i].angle > 368640.0f) {
                angle = (int32_t)(10 * 368640.0f);
            } else if (servoSync[i].angle < -368640.0f) {
                angle = (int32_t)(10 * -368640.0f);
            } else {
                angle = (int32_t)(10 * servoSync[i].angle);
            }

            uint8_t *p = content + 3 + i * 15;
            p[0] = servoSync[i].id;
            p[1] = angle & 0xFF;
            p[2] = (angle >> 8) & 0xFF;
            p[3] = (angle >> 16) & 0xFF;
            p[4] = (angle >> 24) & 0xFF;
            p[5] = servoSync[i].interval_multi & 0xFF;
            p[6] = (servoSync[i].interval_multi >> 8) & 0xFF;
            p[7] = (servoSync[i].interval_multi >> 16) & 0xFF;
            p[8] = (servoSync[i].interval_multi >> 24) & 0xFF;
            p[9] = servoSync[i].t_acc & 0xFF;
            p[10] = (servoSync[i].t_acc >> 8) & 0xFF;
            p[11] = servoSync[i].t_dec & 0xFF;
            p[12] = (servoSync[i].t_dec >> 8) & 0xFF;
            p[13] = servoSync[i].power & 0xFF;
            p[14] = (servoSync[i].power >> 8) & 0xFF;
        }
        break;

    case MODE_SET_SERVO_ANGLE_MTURN_BY_VELOCITY:
        /* 同步命令设置舵机的角度(多圈模式, 指定转速) */
        size = 3 + servo_count * 13;
        content = (uint8_t *)pvPortMalloc(size);
        if (content == NULL) return SERVO_STATUS_FAIL;

        content[0] = 15;
        content[1] = 13;
        content[2] = servo_count;

        for (int i = 0; i < servo_count; i++) {
            int32_t angle;
            if (servoSync[i].angle > 368640.0f) {
                angle = (int32_t)(10 * 368640.0f);
            } else if (servoSync[i].angle < -368640.0f) {
                angle = (int32_t)(10 * -368640.0f);
            } else {
                angle = (int32_t)(10 * servoSync[i].angle);
            }
            uint16_t velocity = (uint16_t)(10.0f * servoSync[i].velocity);

            uint8_t *p = content + 3 + i * 13;
            p[0] = servoSync[i].id;
            p[1] = angle & 0xFF;
            p[2] = (angle >> 8) & 0xFF;
            p[3] = (angle >> 16) & 0xFF;
            p[4] = (angle >> 24) & 0xFF;
            p[5] = velocity & 0xFF;
            p[6] = (velocity >> 8) & 0xFF;
            p[7] = servoSync[i].t_acc & 0xFF;
            p[8] = (servoSync[i].t_acc >> 8) & 0xFF;
            p[9] = servoSync[i].t_dec & 0xFF;
            p[10] = (servoSync[i].t_dec >> 8) & 0xFF;
            p[11] = servoSync[i].power & 0xFF;
            p[12] = (servoSync[i].power >> 8) & 0xFF;
        }
        break;

    case MODE_Query_SERVO_Monitor:
        /* 同步命令读取舵机的数据 */
        size = 3 + servo_count;
        content = (uint8_t *)pvPortMalloc(size);
        if (content == NULL) return SERVO_STATUS_FAIL;

        content[0] = 22;
        content[1] = 1;
        content[2] = servo_count;
        for (int i = 0; i < servo_count; i++) {
            content[3 + i] = servoSync[i].id;
        }
        break;

    default:
        return SERVO_STATUS_ERROR;
    }

    // 发送请求包（根据size是否超过255决定isSync）
    uint8_t isSync = (size > 255) ? 1 : 0;
    status = Servo_SendPackage_Common(SERVO_CMD_SET_SERVO_SyncCommand, size, content, isSync);

    // 如果是监控模式，接收响应数据
    if ((status == SERVO_STATUS_SUCCESS) && (servo_mode == MODE_Query_SERVO_Monitor)) {
        status = Servo_SyncServoMonitor(servo_count, servodata);
    }

    // 释放动态分配的内存
    vPortFree(content);

    return status;
}

static SERVO_STATUS servo_sync_monitor_impl(uint8_t servo_count, ServoData servodata[]) {
    Package_t pkg;
    SERVO_STATUS status;
    uint8_t packet_count = 0;

    while (packet_count < servo_count) {
        status = Servo_Sync_RecvPackage(&pkg);
        if (status != SERVO_STATUS_SUCCESS) {
            return status;
        }

        servodata[packet_count].id = pkg.content[0];
        servodata[packet_count].voltage = (int16_t)((pkg.content[2] << 8) | pkg.content[1]);
        servodata[packet_count].current = (int16_t)((pkg.content[4] << 8) | pkg.content[3]);
        servodata[packet_count].power = (int16_t)((pkg.content[6] << 8) | pkg.content[5]);

        // 温度转换（原公式，添加保护）
        uint16_t adc_temp = (uint16_t)((pkg.content[8] << 8) | pkg.content[7]);
        if ((adc_temp > 0U) && (adc_temp < 4096U)) { // 避免除零/负对数
            double temp = adc_temp;
            double kelvin = 1.0 / (log(temp / (4096.0 - temp)) / 3435.0 + 1.0 / (273.15 + 25.0));
            servodata[packet_count].temperature = (int16_t)(kelvin - 273.15);
        } else {
            servodata[packet_count].temperature = 0;
        }

        servodata[packet_count].status = pkg.content[9];

        /* 先按 uint32 拼接再转 int32, 避免有符号左移溢出(UB) */
        int32_t raw_angle = (int32_t)(((uint32_t)pkg.content[13] << 24) | ((uint32_t)pkg.content[12] << 16) |
                                      ((uint32_t)pkg.content[11] << 8) | ((uint32_t)pkg.content[10]));
        servodata[packet_count].angle = (float)raw_angle / 10.0f;
        servodata[packet_count].circle_count = (int16_t)((pkg.content[15] << 8) | pkg.content[14]);

        packet_count++;
        memset(&pkg, 0, sizeof(Package_t));
    }

    return SERVO_STATUS_SUCCESS;
}

/* ==========================================================================
 * 事务包装: 读类命令串行化(说明见文件开头)
 * 包装函数与 servo_protocol.h 的公共声明一致, 实现本体为上面的 *_impl。
 * ========================================================================== */

SERVO_STATUS Servo_Ping(uint8_t servo_id) {
    servo_txn_lock();
    SERVO_STATUS status = servo_ping_impl(servo_id);
    servo_txn_unlock();
    return status;
}

SERVO_STATUS Servo_ResetUserData(uint8_t servo_id) {
    servo_txn_lock();
    SERVO_STATUS status = servo_reset_user_data_impl(servo_id);
    servo_txn_unlock();
    return status;
}

SERVO_STATUS Servo_ReadData(uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t *size) {
    servo_txn_lock();
    SERVO_STATUS status = servo_read_data_impl(servo_id, address, value, size);
    servo_txn_unlock();
    return status;
}

SERVO_STATUS Servo_WriteData(uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t size) {
    servo_txn_lock();
    SERVO_STATUS status = servo_write_data_impl(servo_id, address, value, size);
    servo_txn_unlock();
    return status;
}

SERVO_STATUS Servo_QueryServoAngle(uint8_t servo_id, float *angle) {
    servo_txn_lock();
    SERVO_STATUS status = servo_query_angle_impl(servo_id, angle);
    servo_txn_unlock();
    return status;
}

SERVO_STATUS Servo_QueryServoAngleMTurn(uint8_t servo_id, float *angle) {
    servo_txn_lock();
    SERVO_STATUS status = servo_query_angle_mturn_impl(servo_id, angle);
    servo_txn_unlock();
    return status;
}

SERVO_STATUS Servo_Monitor(uint8_t servo_id, ServoData servodata[]) {
    servo_txn_lock();
    SERVO_STATUS status = servo_monitor_impl(servo_id, servodata);
    servo_txn_unlock();
    return status;
}

SERVO_STATUS Servo_SyncCommand(uint8_t servo_count, uint8_t servo_mode, Sync_ServoData servoSync[]) {
    servo_txn_lock();
    SERVO_STATUS status = servo_sync_command_impl(servo_count, servo_mode, servoSync);
    servo_txn_unlock();
    return status;
}

SERVO_STATUS Servo_SyncServoMonitor(uint8_t servo_count, ServoData servodata[]) {
    servo_txn_lock();
    SERVO_STATUS status = servo_sync_monitor_impl(servo_count, servodata);
    servo_txn_unlock();
    return status;
}

/**
 * @file sim_config.h
 * @brief 主机端 nav 仿真：被控对象 / 定位测量参数定义与默认值
 *
 * 单位约定（与固件一致）：
 *   位置 cm、航向 deg、线速度 cm/s、角速度 deg/s、加速度 cm/s²、时间 ms。
 *
 * 仿真链路：
 *   nav_core.c（真实控制代码，5ms/拍）
 *     -> MotionControl_SetVelocity(vx_body, vy_body, w)
 *     -> [sim_platform.c] 命令历史环形缓冲 + 纯延迟（模拟电机总线/驱动器端到端延迟）
 *     -> [sim_plant.c] 真实 Kinematics_Inverse 逆运动学 -> 整数 RPM 量化（复刻 MotionControl）
 *        -> 电机加速度斜坡限幅 -> 轮速一阶滞后 tau -> 正运动学还原机体系速度
 *        -> 打滑系数 -> 车体真实位姿积分（地面里程）
 *     -> [sim_ops.c] OPS 测量（更新率/延迟/量化/噪声）-> 复刻 nav_local.c 的 Loc_Get
 *     -> 回到 nav_core.c
 */

#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* ===== 电机命令链（可对比电机总线修复前后）===== */
    float cmd_delay_ms; /* 命令端到端纯延迟 (ms)：0/10/20/50/100 */
    float wheel_tau_s; /* 轮速一阶滞后时间常数 (s)：机械+速度环，默认 0.06 */
    int rpm_quantize; /* 1=复刻真实 MotionControl 的整数 RPM 截断量化（低速度分辨率受限） */
    int motor_acc_limit; /* 1=电机驱动器内部加速度斜坡限幅（由 nav 下发的 car_acc 折算） */
    float motor_acc_scale; /* 电机实际加速度 / nav 下发加速度，默认 1.0（Emm 档位换算误差约 ±2%） */
    /* STOP 命令（0xFE 立即停止）：驱动器主动制动，与普通速度斜坡不同。
     * 真实硬件上电机立即断力矩/抱死，残余位移主要是轮地滑移/机械柔度；
     * 用"高减速度 + 短时间常数"建模，默认约 1cm 停车距离，具体数值是模型不确定项。 */
    float stop_decel_cm_s2; /* STOP 命令等效减速度 (cm/s²) */
    float stop_tau_s; /* STOP 后轮速一阶滞后时间常数 (s) */

    /* ===== 地面接触 ===== */
    float slip; /* 轮地打滑比例 0~0.10：车轮转动的 (1-slip) 转化为地面位移 */

    /* ===== OPS 平面定位测量 ===== */
    float ops_rate_hz; /* 20 / 50 / 100 */
    float ops_delay_ms; /* 测量+传输延迟 (ms) */
    float ops_pos_quant_cm; /* 位置量化，OPS 标称 0.1mm = 0.01cm */
    float ops_yaw_quant_deg; /* 角度量化 */
    float ops_pos_noise_cm; /* 位置高斯噪声 sigma (cm) */
    float ops_yaw_noise_deg; /* 角度高斯噪声 sigma (deg) */
    uint32_t seed; /* 噪声随机种子（固定保证可复现） */

    /* ===== 控制回路时序 ===== */
    int nav_period_ms; /* 导航控制周期 (ms)，固定 5（与 NAV_UPDATE_TIME 一致） */
    int jitter_max_ms; /* 每个控制周期随机额外延迟上限 (ms)，模拟命令队列阻塞造成的周期抖动 */
    float jitter_prob; /* 发生抖动的概率 0~1（jitter_max_ms 用） */
    int max_time_ms; /* 单场景仿真时长上限 (ms) */
    int hold_after_complete_ms; /* 最后一腿 COMPLETE 后继续仿真多久，用于观察停稳 */
} SimPlantCfg;

void SimCfg_Default(SimPlantCfg *cfg);

#ifdef __cplusplus
}
#endif

#endif /* SIM_CONFIG_H */

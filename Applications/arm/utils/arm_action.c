/**
 * @file arm_action.c
 * @author T-Car Team
 * @brief 机械臂任务动作 实现
 *
 * 物料盘由总线舵机驱动做直线运动: 夹爪到物料盘的取放位置是固定的,
 * 由物料盘把目标格位送到夹爪下方(Arm_PadSetSlot), 机械臂不做物料盘格位的水平偏移;
 * 暂存区/粗加工区的格位则由机械臂按逐点标定的预设位姿(伸展长度/基座角度/抬升高度)走位。
 *
 * 每个对外动作都返回 bool: 任一步骤失败立即返回 false, 由 mission 任务层把本轮判失败,
 * 避免机械臂处于不确定位姿时继续执行后续动作。原料抓取的颜色等待带超时,
 * 不会因为树莓派没识别到颜色而无限阻塞任务。
 * 可调参数见 arm_config.h / Applications/arm。
 */

#include "arm_action.h"
#include "arm_config.h"
#include "arm_ctrl.h"
#include "arm_motion.h"
#include "log.h"
#include "cmsis_os2.h"
#include <math.h>

/*============================ 动作内部偏移参数(待标定) ============================*/
/**
 * 工位目标位姿(伸展长度/基座角度/抬升高度)统一在 arm_config.h 的"动作目标位姿(待标定)"区配置
 * (ARM_POSE_*_INIT 初始化宏), 这里只做静态绑定, 便于动作代码按值/按址使用:
 * 头文件不能直接放 static const 数据(每个包含它的编译单元都会各生成一份副本,
 * 且未使用时会触发未引用告警), 故采用"头文件给初始化宏 + 本文件 static const 绑定"。
 */
static const ArmPose_t s_pose_matl_grab = ARM_POSE_MATL_GRAB_INIT; // 原料区抓取位
static const ArmPose_t s_pose_pad = ARM_POSE_PAD_INIT; // 物料盘取放位(固定, 上方安全高度)
static const ArmPose_t s_pose_init = ARM_POSE_INIT_INIT; // 机械臂初始状态(行驶/待机位)

/**
 * 各动作的速度/加速度 profile 绑定(arm_config.h 给 ARM_PROFILE_xxx_INIT 宏):
 * 动作开始时调用 Arm_SetMotionProfile 选用对应 profile, 动作内部所有移动都用它,
 * 实现"每个机械臂动作的速度/加速度互相解耦"(初值都等于各轴默认速度)。
 */
static const ArmMotionProfile_t s_profile_load_pad = ARM_PROFILE_LOAD_PAD_INIT;
static const ArmMotionProfile_t s_profile_unload_process = ARM_PROFILE_UNLOAD_PROCESS_INIT;
static const ArmMotionProfile_t s_profile_unload_storage = ARM_PROFILE_UNLOAD_STORAGE_INIT;
static const ArmMotionProfile_t s_profile_load_from_process = ARM_PROFILE_LOAD_FROM_PROCESS_INIT;
static const ArmMotionProfile_t s_profile_load_from_storage = ARM_PROFILE_LOAD_FROM_STORAGE_INIT;
static const ArmMotionProfile_t s_profile_init_pose = ARM_PROFILE_INIT_POSE_INIT;
#if ARM_CAL_PREP_ENABLE
static const ArmMotionProfile_t s_profile_cal_prep = ARM_PROFILE_CAL_PREP_INIT;
#endif

/**
 * 粗加工区/暂存区放置位预设表: 每个格位(以及暂存区每层)都是逐点实车标定后填在
 * arm_config.h 的 ARM_POSE_PROCESS_SLOTn_INIT / ARM_POSE_STORAGE_SLOTn_Lm_INIT 里,
 * 放置/取料动作直接查表调用, 不再用"中心位姿 + 格距位移"换算(实测偏移法效果不好)。
 * 表下标: 粗加工区 [slot-1]; 暂存区 [slot-1][layer]。
 */
#if ARM_SLOT_NUM != 3
#error "放置位预设表按 ARM_SLOT_NUM=3 定义, 修改格位数量时需同步更新 arm_config.h 预设宏与本表"
#endif
static const ArmPose_t s_place_process[ARM_SLOT_NUM] = {
    ARM_POSE_PROCESS_SLOT1_INIT,
    ARM_POSE_PROCESS_SLOT2_INIT,
    ARM_POSE_PROCESS_SLOT3_INIT,
};
static const ArmPose_t s_place_storage[ARM_SLOT_NUM][2] = {
    {ARM_POSE_STORAGE_SLOT1_L0_INIT, ARM_POSE_STORAGE_SLOT1_L1_INIT},
    {ARM_POSE_STORAGE_SLOT2_L0_INIT, ARM_POSE_STORAGE_SLOT2_L1_INIT},
    {ARM_POSE_STORAGE_SLOT3_L0_INIT, ARM_POSE_STORAGE_SLOT3_L1_INIT},
};

/*============================ 内部工具 ============================*/

/* 带日志钳位描述符：below/above 为**完整**格式串（含 log.h 的级别前缀与行尾）。
 *
 * 为什么不用 logWarning(desc->below_fmt, ...)：logWarning 展开到 logFormat，后者用
 * `text " " fmt "" LOG_END` 做**编译期**字符串拼接，fmt 必须是字符串字面量。传入运行期
 * const char* 会拼出 `"[WARNING]%s:" " " desc->below_fmt "" "\r\n"`，即字面量与变量直接
 * 相邻（中间无运算符），是非法 C：AC5 报 #18 expected ")" / #127 expected a statement。
 * 这里改为直接调用 logWrite（logWarning 的展开主体），描述符在初始化时按**相同的 log.h
 * 宏**把 WARNING_TEXT 前缀与 LOG_END 后缀拼进完整串，因此输出与
 * `logWarning("...", ...)` 逐字节等价：级别 LOG_WRANING、前缀、TAG、格式串、参数均不变。
 * limit_as_int=true 时用 %d 输出限位（base 字段），否则用 %.1f/%.2f。 */
typedef struct {
    const char *below_fmt;
    const char *above_fmt;
    bool        limit_as_int;
} ArmClampLogDesc_t;

static const ArmClampLogDesc_t k_extend_log = {
    WARNING_TEXT " " "arm: extend %.2fmm below min %.1fmm, clamp to min" LOG_END,
    WARNING_TEXT " " "arm: extend %.2fmm above max %.1fmm, clamp to max" LOG_END, false};
static const ArmClampLogDesc_t k_lift_log = {
    WARNING_TEXT " " "arm: lift %.2fmm below min %.1fmm, clamp to min" LOG_END,
    WARNING_TEXT " " "arm: lift %.2fmm above max %.1fmm, clamp to max" LOG_END, false};
static const ArmClampLogDesc_t k_base_log = {
    WARNING_TEXT " " "arm: base %.1fdeg below min %d, clamp to min" LOG_END,
    WARNING_TEXT " " "arm: base %.1fdeg above max %d, clamp to max" LOG_END, true};
static const ArmClampLogDesc_t k_yaw_log = {
    WARNING_TEXT " " "arm: yaw correction %.2fdeg clamped to %.2fdeg" LOG_END,
    WARNING_TEXT " " "arm: yaw correction %.2fdeg clamped to %.2fdeg" LOG_END, false};

/* 先判 min 后判 max（与 pose 三段原顺序一致）；NaN 两比较皆 false → 原值、不告警。
 * 告警在赋值前发出，与原调用点顺序一致。
 * tag 由调用方传入 __FUNCTION__：logWarning 的 LOG_TAG 在调用方展开，直接在本 helper
 * 里调用 logWrite 会取到 "arm_action_clamp_field"，故显式传入以保留原日志 TAG。
 * 注意不能写成 `if (c) logWarning(...); else logWarning(...);`：logWarning 展开为
 * `if (LOG_ENABLE) {...}`，else 会错误绑定到内层 if（悬垂 else），使 limit_as_int=false
 * 分支永不告警；这里每个分支均用 {} 包裹。 */
static void arm_action_clamp_field(float *value, float min, float max,
                                   const ArmClampLogDesc_t *desc, const char *tag) {
    if (*value < min) {
        if (LOG_ENABLE) {
            if (desc->limit_as_int) {
                logWrite(LOG_ALL_OBJ, LOG_WRANING, desc->below_fmt, tag,
                         (double)*value, (int)min);
            } else {
                logWrite(LOG_ALL_OBJ, LOG_WRANING, desc->below_fmt, tag,
                         (double)*value, (double)min);
            }
        }
        *value = min;
    } else if (*value > max) {
        if (LOG_ENABLE) {
            if (desc->limit_as_int) {
                logWrite(LOG_ALL_OBJ, LOG_WRANING, desc->above_fmt, tag,
                         (double)*value, (int)max);
            } else {
                logWrite(LOG_ALL_OBJ, LOG_WRANING, desc->above_fmt, tag,
                         (double)*value, (double)max);
            }
        }
        *value = max;
    }
}

/**
 * @brief 位姿的伸展/升降/基座数值超软限位时钳到限位内(告警后继续)
 * @param pose 目标位姿(可能被原地钳位)
 * @return true=可执行(已在限位内或已钳位)
 * @note 现场策略: 目标超出可达范围时按"最大值(软限位)"执行, 不再判失败;
 *       位姿表因标定误差/占位值略超限时也会钳到限位值继续动作, 只告警。
 */
static bool arm_action_clamp_pose(ArmPose_t *pose) {
    arm_action_clamp_field(&pose->extend_mm, ARM_EXTEND_MIN_MM, ARM_EXTEND_MAX_MM,
                           &k_extend_log, __FUNCTION__);
    arm_action_clamp_field(&pose->lift_mm, ARM_LIFT_MIN_MM, ARM_LIFT_MAX_MM,
                           &k_lift_log, __FUNCTION__);
    arm_action_clamp_field(&pose->base_angle_deg,
                           (float)ARM_BASE_MIN_DEG, (float)ARM_BASE_MAX_DEG,
                           &k_base_log, __FUNCTION__);

    return true;
}

/**
 * @brief 移动到位姿并等待到位(目标超限自动钳位; 某段未到位按实际位置继续, 不判失败)
 * @param pose 目标位姿
 * @return true=已执行完(含"未完全到位但按实际位置继续"的 best effort 情况); false=位姿为空/电机离线
 */
static bool arm_action_move(const ArmPose_t *pose) {
    if (pose == NULL) return false;

    ArmPose_t target = *pose;
    if (!arm_action_clamp_pose(&target)) {
        return false;
    }

    if (!Arm_MovePolarToAndWait(target.extend_mm, target.base_angle_deg, target.lift_mm, ARM_MOVE_TIMEOUT_MS)) {
        logError("arm: move (extend %.1f, angle %.1f, lift %.1f) failed (arm offline/unavailable)",
                 (double)target.extend_mm,
                 (double)target.base_angle_deg,
                 (double)target.lift_mm);
        return false;
    }
    return true;
}

/**
 * @brief 在父位姿基础上只调整抬升高度, 得到新的位姿(保持伸展长度/基座角度不变)
 * @param base     父位姿
 * @param delta_mm 抬升偏移(mm), 正=向上抬升, 负=向下
 * @return 调整后的位姿
 */
static ArmPose_t arm_action_offset_lift(const ArmPose_t *base, float delta_mm) {
    ArmPose_t pose = *base;
    pose.lift_mm += delta_mm;
    return pose;
}

/*============================ 视觉 yaw 误差补偿 ============================*/

/** 视觉标定 yaw 误差的基座补偿角(°): 任务层标定成功后设置, 每个工位开始时清零 */
static float s_yaw_correction_deg = 0.0f;

void Arm_Action_SetYawCorrection(float yaw_err_deg) {
#if ARM_YAW_CORRECTION_ENABLE
    float corr = yaw_err_deg * ARM_YAW_CORRECTION_SIGN;

    arm_action_clamp_field(&corr, -ARM_YAW_CORRECTION_MAX_DEG, ARM_YAW_CORRECTION_MAX_DEG,
                           &k_yaw_log, __FUNCTION__);

    if (corr > -0.01f && corr < 0.01f) corr = 0.0f;
    if (corr == s_yaw_correction_deg) return;

    s_yaw_correction_deg = corr;
    logInfo("arm: yaw correction set to %.2fdeg (visual yaw err %.2fdeg)",
            (double)s_yaw_correction_deg,
            (double)yaw_err_deg);
#else
    (void)yaw_err_deg;
    s_yaw_correction_deg = 0.0f;
#endif
}

/**
 * @brief 给"场地固定"的目标位姿加上视觉 yaw 补偿(基座角度 += 补偿角)
 * @param pose 原始位姿(基于机械零位/车体坐标系)
 * @return 补偿后的位姿
 * @note 只用于原料区/粗加工区/暂存区等场地固定工位; 物料盘随车体转动, 不要用本函数补偿。
 */
static ArmPose_t arm_action_field_pose(const ArmPose_t *pose) {
    ArmPose_t corrected = *pose;
    corrected.base_angle_deg += s_yaw_correction_deg;
    return corrected;
}

float Arm_Action_GetYawCorrection(void) {
    return s_yaw_correction_deg;
}

bool Arm_Action_CalcFieldPose(ArmPlaceArea_t area, uint8_t slot, uint8_t layer, ArmPose_t *pose) {
    if (!Arm_Action_CalcPlacePose(area, slot, layer, pose)) {
        return false;
    }
    *pose = arm_action_field_pose(pose);
    return true;
}

/**
 * @brief 回到机械臂初始状态位(行驶/待机位: 臂收短 + 抬升)
 * @return true=已到位; false=运动失败
 * @note 位姿在 arm_config.h 的 ARM_POSE_INIT_* (占位值需标定, `arm preset init` 对位);
 *       回零完成后回零任务也会走到该位; 每次小车移动前由任务层调用, 避免带长臂/低姿态行驶。
 *       只动三轴, 不动夹爪/物料盘。
 */
bool Arm_Action_MoveToInitPose(void) {
    Arm_SetMotionProfile(&s_profile_init_pose);

    logInfo("arm: move to initial pose (extend %.1f, angle %.1f, lift %.1f)",
            (double)s_pose_init.extend_mm,
            (double)s_pose_init.base_angle_deg,
            (double)s_pose_init.lift_mm);
    return arm_action_move(&s_pose_init);
}

/**
 * @brief 放料后垂直向上抬升撤离(限幅到升降行程内)
 * @param placed 刚放料完成的位姿
 * @return true=抬升到位; false=抬升运动失败
 */
static bool arm_action_retract_up(ArmPose_t placed) {
    ArmPose_t retract = arm_action_offset_lift(&placed, ARM_PLACE_LIFT_MM);
    if (retract.lift_mm > ARM_LIFT_MAX_MM) {
        retract.lift_mm = ARM_LIFT_MAX_MM;
    }
    return arm_action_move(&retract);
}

/**
 * @brief 校验物料盘格位号
 */
static bool arm_action_check_pad_slot(uint8_t pad_slot) {
    if (pad_slot < 1 || pad_slot > ARM_PAD_SLOT_NUM) {
        logError("arm: invalid pad slot %u (expect 1~%u)", (unsigned)pad_slot, (unsigned)ARM_PAD_SLOT_NUM);
        return false;
    }
    return true;
}

/*============================ 内部动作 ============================*/

/**
 * @brief 原料区抓取: 移动到抓取位, 等待目标颜色稳定后闭合夹爪(停在夹取位, 不抬升)
 * @param color 目标颜色
 * @return true=已夹取(臂仍在夹取位); false=颜色非法/颜色识别超时/运动失败
 *
 * 颜色判定: 树莓派只回传一个颜色; 连续两次检测到目标颜色才算稳定;
 * 超过 ARM_GRAB_COLOR_TIMEOUT_MS 未识别到目标颜色则判失败(避免任务无限阻塞)。
 * 识别会话: 抓取期间保持树莓派连续上报颜色(采样之间不补发 0x11, 不用每次重新颜色锁定),
 *           抓完当前目标颜色后再补发 0x11 结束会话。
 * 张开角: 原料区用标定大张开角 ARM_CAL_GRIPPER_OPEN_DEG(-110), 不用常规 -55;
 *         张开后再移动到抓取位(水平 + **单次**下降), 不再先降一档安全高度、再降一档到抓取位。
 * 抬升: 本动作不做抬升, 由调用方(上盘/撤离)一次性"先升到安全高度再水平移动",
 *       避免"先抬 60mm、紧接着上盘移动又抬到盘安全高度"的连续两次抬升。
 */
static bool arm_action_pick_matl(RPI_Color_t color) {
    RPI_Color_t detected = COLOR_UNKNOWN;
    uint8_t hit = 0;
    uint32_t start_tick;
    bool session = false;

    if (color <= COLOR_UNKNOWN || color > COLOR_LIGHTBLUE) {
        logError("arm: invalid target color %d", (int)color);
        return false;
    }

    /* 原料区是场地固定工位: 抓取位叠加视觉 yaw 补偿(来自 rpi cal 的朝向误差) */
    ArmPose_t grab = arm_action_field_pose(&s_pose_matl_grab);

    /* 下降前先把夹爪张到与标定一致的 ARM_CAL_GRIPPER_OPEN_DEG(-110), 避免手指带着闭合/半开
     * 状态下降撞到物料/平台; 张爪与"移动到抓取位"并行: 先只下发张爪命令, 到位后再确认 */
    if (!Arm_GripperStartAngle(ARM_CAL_GRIPPER_OPEN_DEG)) {
        return false;
    }
    if (!arm_action_move(&grab)) {
        return false;
    }
    if (!Arm_GripperWaitArrive()) {
        return false;
    }

    /* 到达抓取位后开始颜色识别会话: 只下发一次识别命令, 抓取期间树莓派保持连续上报,
     * 采样时不再"发识别命令 + 补 0x11", 省掉每次重新颜色锁定的时间 */
    session = RPI_ColorSessionBegin();
    if (!session) {
        logWarning("arm: color session begin failed, fallback to one-shot detect");
    }

    start_tick = osKernelGetTickCount();
    for (;;) {
        bool got = session
                       ? RPI_ColorSessionRead(&detected, (hit == 0) ? RPI_RSP_TIMEOUT_MS : RPI_COLOR_SAMPLE_TIMEOUT_MS)
                       : RPI_Detect_Color(&detected);
        if (got && detected == color) {
            if (++hit >= 2) break;
        } else {
            hit = 0;
        }

        if ((osKernelGetTickCount() - start_tick) >= ARM_GRAB_COLOR_TIMEOUT_MS) {
            logError("arm: target color %d not detected in %ums (detected=%d)",
                     (int)color,
                     (unsigned)ARM_GRAB_COLOR_TIMEOUT_MS,
                     (int)detected);
            if (session) {
                RPI_ColorSessionEnd();
            }
            return false;
        }
        osDelay(1);
    }

    if (!Arm_GripperSetAngle(ARM_GRIPPER_CLOSE_DEG)) {
        if (session) {
            RPI_ColorSessionEnd();
        }
        return false;
    }

    /* 抓完当前设定颜色: 补发 0x11 结束本轮连续上报; 下一次抓取到达新抓取位后
     * 再由 RPI_ColorSessionBegin 重新下发识别命令(避免在这颗物料还在夹爪里时提前识别,
     * 把刚抓到的颜色锁存下来导致下一颗颜色对不上) */
    if (session) {
        RPI_ColorSessionEnd();
    }

    /* 不在这里抬升: 保持夹取位直接返回, 由调用方的移动一次性"先升到安全高度再水平",
     * 否则会"先抬 60mm、紧接着上盘移动又抬到盘安全高度", 看起来抬升两次 */
    return true;
}

/**
 * @brief 启动物料盘换格(非阻塞): 校验盘位并下发换格命令
 * @param pad_slot 物料盘盘位号
 * @return true=换格命令已下发; false=盘位非法/命令下发失败
 * @note 可提前调用, 让物料盘转动与机械臂"抓取/移臂"并行, 节约时间;
 *       随后用 arm_action_pad_release_finish() 或 arm_action_pad_pick() 校验到位。
 */
static bool arm_action_pad_start(uint8_t pad_slot) {
    if (!arm_action_check_pad_slot(pad_slot)) {
        return false;
    }
    return Arm_PadSetSlotStart(pad_slot);
}

/**
 * @brief 物料盘放料(换格已提前启动): 移到盘上方安全高度, 确认到位后下降放料并抬回
 * @return true=放料完成且抬回安全高度; false=运动失败/物料盘未到位
 * @note 物料盘换格命令需已由 arm_action_pad_start() 提前下发; 盘转动与机械臂移向物料盘并行。
 */
static bool arm_action_pad_release_finish(void) {
    /* 此时物料盘可能还在转, 机械臂先在安全高度就位, 不会与物料盘干涉 */
    if (!arm_action_move(&s_pose_pad)) {
        return false;
    }
    /* 下降放料前校验物料盘已到位(角度回读确认), 避免与物料盘/其它格位干涉 */
    if (!Arm_PadWaitArrive(ARM_PAD_SETTLE_TIMEOUT_MS)) {
        return false;
    }

    ArmPose_t release = arm_action_offset_lift(&s_pose_pad, -PAD_REACH_MM);
    if (!arm_action_move(&release)) {
        return false;
    }
    if (!Arm_GripperSetAngle(ARM_GRIPPER_OPEN_DEG)) {
        return false;
    }

    /* 放料后先抬起, 再允许横向移动 */
    return arm_action_move(&s_pose_pad);
}

/**
 * @brief 物料盘取料: 先启动换格, 同时移到物料盘上方安全高度, 确认到位后再下降夹取
 * @param pad_slot 物料盘格位号
 * @return true=取料完成且抬回安全高度; false=格位非法/运动失败/物料盘未到位
 * @note 与放料相同: 物料盘换格与机械臂移到安全高度并行, 下降前再确认到位;
 *       取料前先把夹爪张到常规张开角 ARM_GRIPPER_OPEN_DEG(-55), 再下降闭合夹取。
 */
static bool arm_action_pad_pick(uint8_t pad_slot) {
    if (!arm_action_check_pad_slot(pad_slot)) {
        return false;
    }

    /* 物料盘取料前先把夹爪张到常规张开角 ARM_GRIPPER_OPEN_DEG(-55):
     * 张爪与"启动换格 + 移到盘上方"并行, 先只下发张爪命令, 到位后再确认(省一次串行等待) */
    if (!Arm_GripperStartAngle(ARM_GRIPPER_OPEN_DEG)) {
        return false;
    }

    /* 先启动换格, 让物料盘转动与机械臂移向物料盘上方并行执行 */
    if (!Arm_PadSetSlotStart(pad_slot)) {
        return false;
    }
    /* 此时物料盘可能还在转, 机械臂先在安全高度就位, 不会与物料盘干涉 */
    if (!arm_action_move(&s_pose_pad)) {
        return false;
    }
    if (!Arm_GripperWaitArrive()) {
        return false;
    }
    /* 下降取料前校验物料盘已到位(角度回读确认) */
    if (!Arm_PadWaitArrive(ARM_PAD_SETTLE_TIMEOUT_MS)) {
        return false;
    }

    ArmPose_t reach = arm_action_offset_lift(&s_pose_pad, -PAD_REACH_MM);
    if (!arm_action_move(&reach)) {
        return false;
    }
    if (!Arm_GripperSetAngle(ARM_GRIPPER_CLOSE_DEG)) {
        return false;
    }

    /* 取料后先抬起, 再允许横向移动 */
    return arm_action_move(&s_pose_pad);
}

/**
 * @brief 场地格位取料: 移到格位 -> 闭合夹爪(不单独抬升, 抬升由后续移动一次完成)
 * @param target 格位目标位姿(与放置动作共用同一套格位/层位姿计算)
 * @return true=已夹取(臂仍在取料位); false=运动失败
 * @note 取料点与放置动作的放料点一致(同一目标位姿); 夹取后保持取料位返回,
 *       由调用方的上盘移动"先升到安全高度再水平"一次性抬升撤离,
 *       避免"先抬 ARM_PLACE_LIFT_MM、紧接着上盘移动又抬到盘安全高度"的连续两次抬升。
 */
static bool arm_action_field_pick(const ArmPose_t *target) {
    if (target == NULL) {
        return false;
    }

    /* 场地固定工位: 取料位叠加视觉 yaw 补偿(来自 rpi cal 的朝向误差) */
    ArmPose_t field = arm_action_field_pose(target);

    /* 夹爪先张开再靠近, 避免带着闭爪/残料撞到格位上的物料;
     * 张爪与"移动到格位"并行: 先只下发张爪命令, 到位后再确认 */
    if (!Arm_GripperStartAngle(ARM_GRIPPER_OPEN_DEG)) {
        return false;
    }

    if (!arm_action_move(&field)) {
        return false;
    }
    if (!Arm_GripperWaitArrive()) {
        return false;
    }
    if (!Arm_GripperSetAngle(ARM_GRIPPER_CLOSE_DEG)) {
        return false;
    }

    /* 不在这里抬升: 保持取料位直接返回, 抬升由调用方的移动一次性完成, 避免连抬两次 */
    return true;
}

/*============================ 对外动作 ============================*/

bool Arm_Action_CalcPlacePose(ArmPlaceArea_t area, uint8_t slot, uint8_t layer, ArmPose_t *pose) {
    if (pose == NULL) {
        logError("arm: null place pose output");
        return false;
    }
    if (slot < 1 || slot > ARM_SLOT_NUM) {
        logError("arm: invalid place slot %u (expect 1~%u)", (unsigned)slot, (unsigned)ARM_SLOT_NUM);
        return false;
    }

    /* 直接查逐点标定的预设表(不再用位移偏移换算): 粗加工区按格位, 暂存区按格位+层 */
    switch (area) {
    case ARM_PLACE_AREA_PROCESSING:
        *pose = s_place_process[slot - 1];
        return true;

    case ARM_PLACE_AREA_STORAGE:
        if (layer > 1) {
            logError("arm: invalid storage layer %u (expect 0/1)", (unsigned)layer);
            return false;
        }
        *pose = s_place_storage[slot - 1][layer];
        return true;

    default:
        logError("arm: invalid place area %d", (int)area);
        return false;
    }
}

bool Arm_Action_LoadPad(RPI_Color_t color, uint8_t pad_slot) {
    Arm_SetMotionProfile(&s_profile_load_pad);

    /* 抓料前先启动物料盘换格: 物料盘转动与"去原料区抓取"并行, 抓完回到盘上时盘已到位, 节约时间 */
    if (!arm_action_pad_start(pad_slot)) {
        return false;
    }
    if (!arm_action_pick_matl(color)) {
        return false;
    }
    /* 夹取后由 pad_release_finish 的移动一次性完成抬升(上升动作先升到盘安全高度、再水平移动),
     * 不再在抓取动作里单独抬一次, 动作更连贯 */
    return arm_action_pad_release_finish();
}

bool Arm_Action_UnloadToProcessing(uint8_t pad_slot, uint8_t process_slot, uint8_t next_pad_slot) {
    ArmPose_t target;

    Arm_SetMotionProfile(&s_profile_unload_process);

    /* 目标点 = 粗加工区中心格 + 格位直线偏移换算(与调试工具共用 Arm_Action_CalcPlacePose) */
    if (!Arm_Action_CalcPlacePose(ARM_PLACE_AREA_PROCESSING, process_slot, 0, &target)) {
        return false;
    }
    /* 场地固定工位: 放置位叠加视觉 yaw 补偿(来自 rpi cal 的朝向误差) */
    target = arm_action_field_pose(&target);

    if (!arm_action_pad_pick(pad_slot)) {
        return false;
    }

    /* 取到料后物料盘空闲: 立即预启动下一盘位换格, 让换格与"移臂到场地放料"并行, 节约时间 */
    if (next_pad_slot != 0 && !arm_action_pad_start(next_pad_slot)) {
        return false;
    }

    if (!arm_action_move(&target)) {
        return false;
    }

    if (!Arm_GripperSetAngle(ARM_GRIPPER_OPEN_DEG)) {
        return false;
    }

    /* 放料后垂直抬升撤离, 避免移臂/转基座时刮碰已放物料 */
    return arm_action_retract_up(target);
}

bool Arm_Action_UnloadToStorage(uint8_t pad_slot, uint8_t storage_slot, uint8_t layer, uint8_t next_pad_slot) {
    ArmPose_t target;

    Arm_SetMotionProfile(&s_profile_unload_storage);

    /* 目标点 = 暂存区中心格 + 格位直线偏移换算 + 叠层高度(与调试工具共用 Arm_Action_CalcPlacePose) */
    if (!Arm_Action_CalcPlacePose(ARM_PLACE_AREA_STORAGE, storage_slot, layer, &target)) {
        return false;
    }
    /* 场地固定工位: 放置位叠加视觉 yaw 补偿(来自 rpi cal 的朝向误差) */
    target = arm_action_field_pose(&target);

    if (!arm_action_pad_pick(pad_slot)) {
        return false;
    }

    /* 取到料后物料盘空闲: 立即预启动下一盘位换格, 让换格与"移臂到场地放料"并行, 节约时间 */
    if (next_pad_slot != 0 && !arm_action_pad_start(next_pad_slot)) {
        return false;
    }

    if (!arm_action_move(&target)) {
        return false;
    }

    if (!Arm_GripperSetAngle(ARM_GRIPPER_OPEN_DEG)) {
        return false;
    }

    /* 放料后垂直抬升撤离, 避免移臂/转基座时刮碰已放物料 */
    return arm_action_retract_up(target);
}

bool Arm_Action_LoadPadFromProcessing(uint8_t process_slot, uint8_t pad_slot) {
    ArmPose_t target;

    Arm_SetMotionProfile(&s_profile_load_from_process);

    /* 目标点 = 粗加工区中心格 + 格位直线偏移换算(与放置动作共用同一套计算) */
    if (!Arm_Action_CalcPlacePose(ARM_PLACE_AREA_PROCESSING, process_slot, 0, &target)) {
        return false;
    }

    /* 先启动物料盘换格: 盘转动与"到场地格位取料"并行, 取到料回到盘上时已到位, 节约时间 */
    if (!arm_action_pad_start(pad_slot)) {
        return false;
    }

    if (!arm_action_field_pick(&target)) {
        return false;
    }

    return arm_action_pad_release_finish();
}

bool Arm_Action_LoadPadFromStorage(uint8_t storage_slot, uint8_t layer, uint8_t pad_slot) {
    ArmPose_t target;

    Arm_SetMotionProfile(&s_profile_load_from_storage);

    /* 目标点 = 暂存区中心格 + 格位直线偏移换算 + 叠层高度(与放置动作共用同一套计算) */
    if (!Arm_Action_CalcPlacePose(ARM_PLACE_AREA_STORAGE, storage_slot, layer, &target)) {
        return false;
    }

    /* 先启动物料盘换格: 盘转动与"到场地格位取料"并行, 取到料回到盘上时已到位, 节约时间 */
    if (!arm_action_pad_start(pad_slot)) {
        return false;
    }

    if (!arm_action_field_pick(&target)) {
        return false;
    }

    return arm_action_pad_release_finish();
}

bool Arm_Action_CalibrationReady(const ArmCalPose_t *pose) {
    if (pose == NULL) {
        logError("arm: null calibration pose");
        return false;
    }

#if ARM_CAL_PREP_ENABLE
    /* 标定预备位姿使用本动作独立的速度/加速度(ARM_PROFILE_CAL_PREP_INIT) */
    Arm_SetMotionProfile(&s_profile_cal_prep);
    const ArmMotionProfile_t *profile = Arm_GetMotionProfile();

    logInfo("arm: calibration prep pose: base %ld deg, extend %.1f mm, lift %.1f mm",
            (long)pose->base_angle_deg,
            (double)pose->extend_mm,
            (double)pose->lift_mm);

    /* 整臂回零是异步执行的: 未完成前下发运动命令会与回零任务并发冲突(零位/姿态可能异常),
     * 标定链路强依赖臂位精度, 因此回零未就绪/失败时直接失败(不再告警后继续)。 */
    if (!Arm_HomeWaitDone(ARM_HOME_READY_TIMEOUT_MS)) {
        logError("arm: homing not ready, refuse calibration prep pose");
        return false;
    }

    /* 标定预备位姿分两段走: **先水平(基座旋转 + 伸展)到位, 再升降到位**,
     * 不在旋转的同时升降(避免带高度斜向摆入标定视野、扫碰旁边结构)。
     * 伸展/旋转/升降的超软限位由 arm_ctrl 对应接口检查, 超限直接返回失败;
     * 负角度由 arm_ctrl 用方向字承载, 不会下发补码。 */
    if (!Arm_ExtendTo(pose->extend_mm, profile->extend_vel, profile->extend_acc)) {
        logError("arm: calibration extend command rejected (extend=%.1f mm)", (double)pose->extend_mm);
        return false;
    }
    if (!Arm_BaseRotateAbs(pose->base_angle_deg, profile->base_vel, profile->base_acc)) {
        logError("arm: calibration base rotate command rejected (base=%ld deg)", (long)pose->base_angle_deg);
        return false;
    }
    if (!Arm_WaitSettledHorizontal(ARM_CAL_SETTLE_TIMEOUT_MS)) {
        logError("arm: calibration prep horizontal (rotate/extend) not settled");
        return false;
    }

    if (!Arm_LiftTo(pose->lift_mm, profile->lift_vel, profile->lift_acc)) {
        logError("arm: calibration lift command rejected (lift=%.1f mm)", (double)pose->lift_mm);
        return false;
    }
    if (!Arm_WaitSettledLift(ARM_CAL_SETTLE_TIMEOUT_MS)) {
        logError("arm: calibration prep lift not settled");
        return false;
    }

#if ARM_CAL_VERIFY_ENABLE
    /* 读回实际基座角度校验: 命令入队成功/位置"稳定"不等于真的转到了目标角度
     * (电机未使能、未回零、命令被丢弃时都会"假到位"), 臂位错误会让视觉标定直接失败
     * 或给出错误修正, 故这里必须兜住。伸展/升降由到位判定兜底。 */
    float actual_deg = 0.0f;
    if (!Arm_MotorGetAngle(ARM_BASE_MOTOR_ID, &actual_deg)) {
        logError("arm: read base angle failed, refuse calibration");
        return false;
    }

    float delta_deg = actual_deg - (float)pose->base_angle_deg;
    if (delta_deg > 180.0f) delta_deg -= 360.0f;
    if (delta_deg < -180.0f) delta_deg += 360.0f;
    if (fabsf(delta_deg) > ARM_CAL_ANGLE_TOL_DEG) {
        logError("arm: calibration prep pose mismatch (target=%ld, actual=%.1f, err=%.1f)",
                 (long)pose->base_angle_deg,
                 (double)actual_deg,
                 (double)delta_deg);
        return false;
    }
    logInfo("arm: calibration prep pose ok (actual base=%.1f deg)", (double)actual_deg);
#endif /* ARM_CAL_VERIFY_ENABLE */
#else
    (void)pose;
#endif /* ARM_CAL_PREP_ENABLE */

    /* 标定前把夹爪张开到固定角度: 相机装在夹爪/末端上时, 闭合夹爪或夹着的物料会挡住
     * 圆盘/圆环视野。此处失败只告警, 不阻断标定(夹爪异常不应导致整轮任务卡死)。
     * 张爪与"翻相机"并行: 先只下发张爪命令, 翻相机(固定等待 ARM_FLIP_SETTLE_MS)期间张爪完成,
     * 最后再确认张爪到位, 省一次串行等待。 */
    bool gripper_ok = Arm_GripperStartAngle(ARM_CAL_GRIPPER_OPEN_DEG);
    if (!gripper_ok) {
        logWarning("arm: calibration gripper open to %d deg failed (continue)", (int)ARM_CAL_GRIPPER_OPEN_DEG);
    }

    /* 三轴确认到位后再翻相机, 供随后的 RPI_Cal 拍摄标定目标:
     * 标定结束后保持该翻转角(后续抓取的颜色识别也用该视角), 不再自动翻回;
     * 默认角只在整臂回零/伸展轴回零前或手动 `arm flip restore` 时恢复。 */
    logInfo("arm: calibration flip camera to %.1f deg", (double)ARM_FLIP_CAL_ANGLE_DEG);
    Arm_FlipSetAngle(ARM_FLIP_CAL_ANGLE_DEG);

    if (gripper_ok) {
        if (Arm_GripperWaitArrive()) {
            logInfo("arm: calibration gripper opened to %d deg", (int)ARM_CAL_GRIPPER_OPEN_DEG);
        } else {
            logWarning("arm: calibration gripper open to %d deg failed (continue)", (int)ARM_CAL_GRIPPER_OPEN_DEG);
        }
    }
    osDelay(ARM_FLIP_SETTLE_MS);

    return true;
}

void Arm_Action_CalibrationFinish(void) {
    /* 标定结束不再恢复相机默认角: 标定角同时作为后续颜色识别的视角(见 arm_config.h"相机翻转舵机"区);
     * 默认角 ARM_FLIP_RESTORE_ANGLE_DEG 只在整臂回零/伸展轴回零前(Arm_CtrlInit / Arm_HomeAxis)
     * 或手动 `arm flip restore` 时恢复。 */
    logInfo("arm: calibration finish, keep camera flip at %.1f deg", (double)ARM_FLIP_CAL_ANGLE_DEG);
}

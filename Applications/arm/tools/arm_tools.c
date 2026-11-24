/**
 * @file arm_tools.c
 * @author T-Car Team
 * @brief 机械臂调试工具: shell 命令 `arm init|home|en|zero|ctrl|pol|pos|gri|pad|flip|preset|presets|act|yaw|key`
 * @note `arm pos` 输出三种数据: 电机角度 / 笛卡尔坐标 xyz / 极坐标(伸展/旋转/抬升)
 *       `arm home [base|lift|extend | motor_id]` 指定单轴固件回零(阻塞到完成), 整臂回零仍用 `arm init`
 *       `arm en on|off` 只使能/失能三轴电机(不回零、不运动)
 *       `arm zero` 把当前位置设为三轴零位(只清零, 不回零/不运动)
 *       `arm flip [deg|cal|restore]` 控制相机翻转舵机(标定/调试角度用, 与标定链路同一接口)
 *       `arm preset grab|pad|init` 直接走到 arm_config.h 的预设工位位姿;
 *       `arm preset process [slot]` / `arm preset storage [slot] [layer]` 按格位/层计算放置位(调试/标定对位用)
 *       `arm act grab|process|storage ...` 跑完整任务动作(含夹爪/物料盘/颜色识别, 与 mission 同一套
 *       Arm_Action_* 接口): 原料抓取上盘、物料盘取料放到粗加工区/暂存区(格位 1~3, 层 0/1);
 *       `arm act pose process|storage [slot] [layer]` 只走到(叠加视觉 yaw 补偿后的)场地放置位
 *       `arm yaw [deg|off]` 查看/设置视觉 yaw 误差的机械臂补偿角(脱离任务流程验证补偿方向/幅度)
 *       `arm key [步长...]` 键盘点动: W/S=伸展伸长/收回, A/D=基座左转/右转, Q/E=升降/下降,
 *       Z/X=夹爪张开/闭合, ^C=退出
 * 可调参数见 arm_config.h / Applications/arm。
 */

#include "shell.h"
#include "shell_cmd_group.h"
#include "log.h"
#include "stdlib.h"
#include "string.h"
#include "arm_config.h"
#include "arm_motion.h"
#include "arm_ctrl.h"
#include "arm_action.h"
#include "cmsis_os2.h"
#include "Events.h"

/**
 * @brief `arm init`: 后台发起整臂回零(立即返回, 约需数十秒)
 */
static void arm_tools_init(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    Arm_Init();
    logPrintln("Arm homing started in background, check progress with 'arm pos'");
}

/**
 * @brief `arm home [axis|motor_id]`: 指定单轴执行固件回零(阻塞到完成/失败)
 * @note 支持轴名(base/lift/extend)或电机总线 ID(5/6/7); 无参数时列出可回零的轴。
 *       与整臂异步回零(`arm init`)互斥: 整臂回零进行中会拒绝执行;
 *       单轴回零会改变该轴位置基准, 其它轴不动, 成功后该轴旧目标记录失效。
 */
static void arm_tools_home(int argc, char *argv[]) {
    if (argc > 2) {
        logPrintln("Usage: arm home [base|lift|extend | motor_id]");
        return;
    }

    if (argc == 1) {
        logPrintln("Usage: arm home [base|lift|extend | motor_id]");
        for (uint8_t i = 0; i < Arm_AxisCount(); i++) {
            logPrintln("  axis %u: %-7s motor ID %u", (unsigned)i, Arm_AxisName(i), (unsigned)Arm_AxisMotorId(i));
        }
        return;
    }

    uint8_t axis = Arm_AxisCount(); /* 非法值: 找不到匹配轴时保持 */
    for (uint8_t i = 0; i < Arm_AxisCount(); i++) {
        if (strcmp(argv[1], Arm_AxisName(i)) == 0) {
            axis = i;
            break;
        }
    }

    if (axis >= Arm_AxisCount()) {
        /* 也允许直接填电机总线 ID */
        char *end = NULL;
        unsigned long motor_id = strtoul(argv[1], &end, 10);
        if (end != argv[1] && *end == '\0') {
            for (uint8_t i = 0; i < Arm_AxisCount(); i++) {
                if ((unsigned long)Arm_AxisMotorId(i) == motor_id) {
                    axis = i;
                    break;
                }
            }
        }
    }

    if (axis >= Arm_AxisCount()) {
        logPrintln("Invalid axis: %s (expect base|lift|extend or motor ID)", argv[1]);
        return;
    }

    logPrintln("Homing axis %u (%s, motor ID%u), blocking until done ...",
               (unsigned)axis,
               Arm_AxisName(axis),
               (unsigned)Arm_AxisMotorId(axis));
    bool ok = Arm_HomeAxis(axis);
    logPrintln("Home %s: %s", Arm_AxisName(axis), ok ? "OK" : "FAILED");
}

/**
 * @brief `arm ctrl <x_mm> <y_mm> <z_mm>`: 笛卡尔坐标移动(异步)
 * @note z_mm 为抬升高度, 0=最底部
 */
static void arm_tools_ctrl(int argc, char *argv[]) {
    if (argc != 4) {
        logPrintln("Usage: arm ctrl <x_mm> <y_mm> <z_mm>");
        logPrintln("z_mm: lift height, 0=bottom, max ARM_LIFT_MAX_MM");
        return;
    }

    float x_mm = atof(argv[1]);
    float y_mm = atof(argv[2]);
    float z_mm = atof(argv[3]);

    Arm_SetMotionProfile(NULL); /* 调试移动用默认速度/加速度(不受上一个任务动作的 profile 影响) */
    if (!Arm_MoveTo(x_mm, y_mm, z_mm)) {
        logPrintln("Failed to move arm to target position");
        return;
    }
    logPrintln("Arm moved to (%.2f, %.2f, z %.2f)\n", x_mm, y_mm, z_mm);
}

/**
 * @brief `arm pol <extend_mm> <angle_deg> <lift_mm>`: 极坐标移动(异步)
 * @note extend_mm 为同步带伸展长度(0=完全收回), 与 `arm pos` 的 extend 读数一致;
 *       lift_mm 为抬升高度, 0=最底部
 */
static void arm_tools_polar(int argc, char *argv[]) {
    if (argc != 4) {
        logPrintln("Usage: arm pol <extend_mm> <base_angle_deg> <lift_mm>");
        logPrintln("extend_mm: belt extension length, 0=fully retracted (same as 'arm pos' extend)");
        logPrintln("angle_deg: 0=home, positive/negative are two sides");
        logPrintln("lift_mm: lift height, 0=bottom, max ARM_LIFT_MAX_MM");
        return;
    }

    float extend_mm = atof(argv[1]);
    float base_angle_deg = atof(argv[2]);
    float lift_mm = atof(argv[3]);

    Arm_SetMotionProfile(NULL); /* 调试移动用默认速度/加速度 */
    if (!Arm_MovePolarTo(extend_mm, base_angle_deg, lift_mm)) {
        logPrintln("Failed to move arm to target position");
        return;
    }
    logPrintln("Arm moved to polar (extend %.2f, angle %.2f, lift %.2f)\n", extend_mm, base_angle_deg, lift_mm);
}

/**
 * @brief `arm pos`: 打印机械臂当前位置的三种数据
 *        1) 机械臂相关电机当前角度(线性轴按逻辑零点折算: 升降从底部、伸展从收回端)
 *        2) 当前机械臂笛卡尔坐标 x/y/z (z=抬升高度)
 *        3) 当前机械臂极坐标: 伸展长度 / 旋转角度 / 抬升高度
 */
static void arm_tools_pos(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    ArmPos_t pos;
    Arm_GetPos(&pos);

    /* 1) 机械臂相关电机当前角度 */
    logPrintln("[arm] pos 1/3 motor angles:");
    for (uint8_t axis = 0; axis < Arm_AxisCount(); axis++) {
        uint8_t motor_id = Arm_AxisMotorId(axis);
        if (pos.motor_angle_valid[axis]) {
            logPrintln(
                "[arm]   ID%d %-6s %8.2f deg", (int)motor_id, Arm_AxisName(axis), (double)pos.motor_angle_deg[axis]);
        } else {
            logPrintln("[arm]   ID%d %-6s unavailable", (int)motor_id, Arm_AxisName(axis));
        }
    }

    /* 2) 笛卡尔坐标: x/y 水平位置, z=抬升高度(0=最底部) */
    logPrintln("[arm] pos 2/3 cartesian:");
    if (pos.cartesian_valid) {
        logPrintln("[arm]   x=%8.2f y=%8.2f z=%8.2f mm",
                   (double)pos.cartesian.x_mm,
                   (double)pos.cartesian.y_mm,
                   (double)pos.cartesian.z_mm);
    } else {
        logPrintln("[arm]   unavailable");
    }

    /* 3) 极坐标: 伸展长度 / 旋转角度 / 抬升高度 */
    logPrintln("[arm] pos 3/3 polar:");
    if (pos.polar_valid) {
        logPrintln("[arm]   extend=%8.2f mm rotate=%8.2f deg lift=%8.2f mm",
                   (double)pos.polar.extend_mm,
                   (double)pos.polar.base_angle_deg,
                   (double)pos.polar.lift_mm);
    } else {
        logPrintln("[arm]   unavailable");
    }
}

/**
 * @brief `arm en <on|off>`: 只使能/失能机械臂三轴电机(不发回零/运动命令)
 * @note off(失能)后电机松轴, 机械臂可能因重力下坠, 调试时先确保臂在安全位置或有人扶。
 */
static void arm_tools_enable(int argc, char *argv[]) {
    if (argc != 2) {
        logPrintln("Usage: arm en <on|off>");
        logPrintln("  on   enable arm motors (hold position)");
        logPrintln("  off  disable arm motors (release shaft, arm may drop)");
        return;
    }

    bool enable;
    if (strcmp(argv[1], "on") == 0) {
        enable = true;
    } else if (strcmp(argv[1], "off") == 0) {
        enable = false;
    } else {
        logPrintln("Usage: arm en <on|off>");
        return;
    }

    if (!Arm_MotorsEnable(enable)) {
        logPrintln("Failed to send %s command to arm motors", enable ? "enable" : "disable");
        return;
    }

    if (enable) {
        logPrintln("Arm motors enable command sent (hold position)");
    } else {
        logPrintln("Arm motors disable command sent (shaft released, arm may drop)");
    }
}

/**
 * @brief `arm zero`: 把机械臂当前位置设为三轴零位(调试/标定用)
 * @note 只下发位置清零命令, 不回零/不运动; 内部先等三轴静止, 清零后读回位置确认。
 */
static void arm_tools_zero(int argc, char *argv[]) {
    if (argc != 1) {
        logPrintln("Usage: arm zero");
        logPrintln("Set current arm position as zero for all 3 motors (no homing/motion)");
        logPrintln("note: lift height still uses the top-home offset, run this at the top zero position");
        return;
    }

    if (!Arm_SetCurrentPosAsZero(ARM_MOVE_TIMEOUT_MS)) {
        logPrintln("Failed to set current arm position as zero");
        return;
    }

    /* 等状态轮询刷新后再读回, 便于确认清零结果 */
    osDelay(300);
    ArmPolarPose_t pose;
    if (Arm_GetPolarPose(&pose)) {
        logPrintln("Arm position set as zero: extend=%.2f mm, rotate=%.2f deg, lift=%.2f mm",
                   (double)pose.extend_mm,
                   (double)pose.base_angle_deg,
                   (double)pose.lift_mm);
    } else {
        logPrintln("Arm position zero command sent (readback unavailable)");
    }
}

/**
 * @brief `arm gri open|close`: 夹爪开合
 */
static void arm_tools_gripper(int argc, char *argv[]) {
    if (argc != 2) {
        logPrintln("Usage: arm gri <open|close>");
        return;
    }

    int angle_deg = (strcmp(argv[1], "close") == 0) ? ARM_GRIPPER_CLOSE_DEG : ARM_GRIPPER_OPEN_DEG;
    bool ok = Arm_GripperSetAngle(angle_deg);
    logPrintln("Gripper set to %d: %s\n", angle_deg, ok ? "OK" : "FAILED");
}

/**
 * @brief `arm pad <slot>`: 物料盘换到指定格位(标定格位角度映射用)
 */
static void arm_tools_pad(int argc, char *argv[]) {
    if (argc != 2) {
        logPrintln("Usage: arm pad <slot>");
        logPrintln("slot: 1~%d", ARM_PAD_SLOT_NUM);
        return;
    }

    uint8_t slot = (uint8_t)atoi(argv[1]);
    bool ok = Arm_PadSetSlot(slot);
    logPrintln("Pad move to slot %d: %s\n", (int)slot, ok ? "OK" : "FAILED");
}

/*============================ 相机翻转舵机调试 ============================*/

/**
 * @brief 打印 `arm flip` 用法与当前翻转舵机配置
 */
static void arm_tools_flip_usage(void) {
    logPrintln("Usage: arm flip [deg | cal | restore]");
    logPrintln("  arm flip            show camera-flip servo config");
    logPrintln("  arm flip <deg>      set flip servo angle (0~180 deg)");
    logPrintln("  arm flip cal        go to ARM_FLIP_CAL_ANGLE_DEG (calibration angle)");
    logPrintln("  arm flip restore    go to ARM_FLIP_RESTORE_ANGLE_DEG (also done before arm init / home extend)");
    logPrintln("config: servo id %d, cal %.1f deg, restore %.1f deg, settle %d ms, enable %d",
               ARM_FLIP_SERVO_ID,
               (double)ARM_FLIP_CAL_ANGLE_DEG,
               (double)ARM_FLIP_RESTORE_ANGLE_DEG,
               ARM_FLIP_SETTLE_MS,
               ARM_FLIP_ENABLE);
}

/**
 * @brief `arm flip [deg|cal|restore]`: 控制相机翻转舵机(标定/调试角度用)
 * @note 直接走 Arm_FlipSetAngle(内部阻塞等待 ARM_FLIP_SETTLE_MS), 与标定链路同一接口;
 *       ARM_FLIP_ENABLE=0 时只打印提示、不动作。
 */
static void arm_tools_flip(int argc, char *argv[]) {
    if (argc == 1) {
        arm_tools_flip_usage();
        return;
    }
    if (argc != 2) {
        arm_tools_flip_usage();
        return;
    }

    float angle_deg;
    if (strcmp(argv[1], "cal") == 0) {
        angle_deg = ARM_FLIP_CAL_ANGLE_DEG;
    } else if (strcmp(argv[1], "restore") == 0) {
        angle_deg = ARM_FLIP_RESTORE_ANGLE_DEG;
    } else {
        /* 数值角度: 允许小数, 范围 0~180 */
        char *end = NULL;
        double value = strtod(argv[1], &end);
        if (end == argv[1] || *end != '\0' || value < 0.0 || value > 180.0) {
            logPrintln("Invalid angle: %s (expect 0~180 deg, or cal/restore)", argv[1]);
            arm_tools_flip_usage();
            return;
        }
        angle_deg = (float)value;
    }

#if ARM_FLIP_ENABLE
    Arm_FlipSetAngle(angle_deg);
    logPrintln("Camera flip servo (id %d) set to %.1f deg", ARM_FLIP_SERVO_ID, (double)angle_deg);
#else
    logPrintln("ARM_FLIP_ENABLE=0: camera flip servo disabled, %.1f deg ignored", (double)angle_deg);
#endif
}

/*============================ 预设位姿调试 ============================*/

/**
 * @brief 固定预设位姿表(无格位/层参数, 直接走位)
 * @note 数值来自 arm_config.h 的"动作目标位姿(待标定)"区(ARM_POSE_*_INIT);
 *       暂存区/粗加工区放置位由 Arm_Action_CalcPlacePose 按格位/层计算(见下方命令)。
 */
typedef struct {
    const char *name; // shell 命令名: arm preset <name>
    const char *desc; // 列表/日志说明
    ArmPose_t pose; // 目标极坐标位姿(伸展长度/基座角度/抬升高度)
} ArmPresetPose_t;

static const ArmPresetPose_t s_fixed_presets[] = {
    {"init", "initial travel/standby pose", ARM_POSE_INIT_INIT},
    {"grab", "material grab pose", ARM_POSE_MATL_GRAB_INIT},
    {"pad", "material pad pose", ARM_POSE_PAD_INIT},
};

#define ARM_FIXED_PRESET_COUNT (sizeof(s_fixed_presets) / sizeof(s_fixed_presets[0]))

/* `arm preset` 用法说明(用字符串数组避免长宏续行; 取值范围在打印处补) */
static const char *const s_preset_usage[] = {
    "Usage:",
    "  arm preset init                       initial travel/standby pose",
    "  arm preset grab                       material grab pose",
    "  arm preset pad                        material pad pose",
    "  arm preset process [slot]             processing place preset, default slot 1",
    "  arm preset storage [slot] [layer]     storage place preset, default slot 1 / layer 0",
    "  arm presets                           list all preset poses",
    "poses: arm_config.h \"action target poses\" section",
};

#define ARM_PRESET_USAGE_COUNT (sizeof(s_preset_usage) / sizeof(s_preset_usage[0]))

/**
 * @brief 打印 `arm preset` 用法(slot/layer 取值范围)
 */
static void arm_tools_preset_usage(void) {
    for (uint8_t i = 0; i < (uint8_t)ARM_PRESET_USAGE_COUNT; i++) {
        logPrintln("%s", s_preset_usage[i]);
    }
    logPrintln("slot: 1~%d (ARM_SLOT_NUM), layer: 0/1", ARM_SLOT_NUM);
}

/**
 * @brief 移动到目标位姿并打印目标/读回坐标
 */
static void arm_tools_move_to(const char *name, const ArmPose_t *pose) {
    Arm_SetMotionProfile(NULL); /* 调试移动用默认速度/加速度 */
    logPrintln("Move arm to preset '%s': polar (extend %.2f, angle %.2f, lift %.2f) mm/deg/mm",
               name,
               (double)pose->extend_mm,
               (double)pose->base_angle_deg,
               (double)pose->lift_mm);
    if (!Arm_MovePolarToAndWait(pose->extend_mm, pose->base_angle_deg, pose->lift_mm, ARM_MOVE_TIMEOUT_MS)) {
        logPrintln("Failed to move arm to preset '%s'", name);
        return;
    }

    /* 打印读回坐标: 标定对位时可直接对比"目标值 vs 实际值" */
    ArmPolarPose_t actual;
    if (Arm_GetPolarPose(&actual)) {
        logPrintln("Arrived preset '%s', actual: extend %.2f mm, rotate %.2f deg, lift %.2f mm",
                   name,
                   (double)actual.extend_mm,
                   (double)actual.base_angle_deg,
                   (double)actual.lift_mm);
    } else {
        logPrintln("Arrived preset '%s' (actual pose unavailable)", name);
    }
}

/**
 * @brief `arm preset ...`: 走到预设位姿(固定位姿或逐点标定的格位/层放置位)
 * @note 只移动三轴, 不做夹爪/物料盘动作; 放置位直接取 arm_action 的预设表, 与任务动作同一份数据。
 */
static void arm_tools_preset(int argc, char *argv[]) {
    if (argc < 2) {
        arm_tools_preset_usage();
        return;
    }

    /* 固定位姿: grab / pad(不接受格位/层参数) */
    for (uint8_t i = 0; i < (uint8_t)ARM_FIXED_PRESET_COUNT; i++) {
        if (strcmp(argv[1], s_fixed_presets[i].name) != 0) {
            continue;
        }
        if (argc != 2) {
            arm_tools_preset_usage();
            return;
        }
        arm_tools_move_to(s_fixed_presets[i].name, &s_fixed_presets[i].pose);
        return;
    }

    /* 放置位: process [slot] / storage [slot] [layer] */
    ArmPlaceArea_t area;
    uint8_t max_argc;
    if (strcmp(argv[1], "process") == 0) {
        area = ARM_PLACE_AREA_PROCESSING;
        max_argc = 3;
    } else if (strcmp(argv[1], "storage") == 0) {
        area = ARM_PLACE_AREA_STORAGE;
        max_argc = 4;
    } else {
        logPrintln("Unknown preset: %s", argv[1]);
        arm_tools_preset_usage();
        return;
    }

    if (argc > max_argc) {
        arm_tools_preset_usage();
        return;
    }

    uint8_t slot = (argc >= 3) ? (uint8_t)atoi(argv[2]) : 1;
    uint8_t layer = (argc >= 4) ? (uint8_t)atoi(argv[3]) : 0;

    ArmPose_t target;
    if (!Arm_Action_CalcPlacePose(area, slot, layer, &target)) {
        logPrintln("Invalid place args: slot=%u layer=%u", (unsigned)slot, (unsigned)layer);
        arm_tools_preset_usage();
        return;
    }

    if (area == ARM_PLACE_AREA_STORAGE) {
        logPrintln("Preset 'storage': slot %u, layer %u", (unsigned)slot, (unsigned)layer);
    } else {
        logPrintln("Preset 'process': slot %u", (unsigned)slot);
    }
    arm_tools_move_to(argv[1], &target);
}

/**
 * @brief `arm presets`: 打印全部预设位姿(固定工位 + 粗加工区/暂存区各格位/层)
 */
static void arm_tools_presets(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    logPrintln("[arm] preset poses (arm_config.h \"action target poses\"):");
    for (uint8_t i = 0; i < (uint8_t)ARM_FIXED_PRESET_COUNT; i++) {
        const ArmPresetPose_t *preset = &s_fixed_presets[i];
        logPrintln("[arm]   %-8s extend=%8.2f angle=%8.2f lift=%8.2f  (%s)",
                   preset->name,
                   (double)preset->pose.extend_mm,
                   (double)preset->pose.base_angle_deg,
                   (double)preset->pose.lift_mm,
                   preset->desc);
    }

    /* 格位/层预设: 逐点标定表, 直接列出全部条目便于核对标定值 */
    ArmPose_t place;
    for (uint8_t slot = 1; slot <= ARM_SLOT_NUM; slot++) {
        if (!Arm_Action_CalcPlacePose(ARM_PLACE_AREA_PROCESSING, slot, 0, &place)) {
            continue;
        }
        logPrintln("[arm]   process%-2u extend=%8.2f angle=%8.2f lift=%8.2f  (args: [slot])",
                   (unsigned)slot,
                   (double)place.extend_mm,
                   (double)place.base_angle_deg,
                   (double)place.lift_mm);
    }
    for (uint8_t slot = 1; slot <= ARM_SLOT_NUM; slot++) {
        for (uint8_t layer = 0; layer <= 1; layer++) {
            if (!Arm_Action_CalcPlacePose(ARM_PLACE_AREA_STORAGE, slot, layer, &place)) {
                continue;
            }
            logPrintln("[arm]   st%u/L%-4u extend=%8.2f angle=%8.2f lift=%8.2f  (args: [slot] [layer])",
                       (unsigned)slot,
                       (unsigned)layer,
                       (double)place.extend_mm,
                       (double)place.base_angle_deg,
                       (double)place.lift_mm);
        }
    }
    logPrintln("[arm] place presets: per-slot/layer calibrated (arm_config.h ARM_POSE_*_SLOTn_*_INIT)");
    logPrintln("[arm] place args: slot 1~%d, layer 0/1", ARM_SLOT_NUM);
}

/*============================ 任务动作调试(完整动作) ============================*/

/**
 * @brief 解析 0~(max) 的十进制无符号参数(带范围校验)
 */
static bool arm_tools_parse_uint(const char *text, uint32_t min_value, uint32_t max_value, uint32_t *value) {
    if (text == NULL || value == NULL || *text == '\0') return false;

    char *end = NULL;
    unsigned long v = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || v < min_value || v > max_value) return false;

    *value = (uint32_t)v;
    return true;
}

/**
 * @brief 解析颜色参数: 支持数字 1~6(RPI_Color_t) 或小写英文名
 * @param text  red/yellow/blue/green/black/lightblue 或 1~6
 * @param color 输出颜色
 * @return true=解析成功
 */
static bool arm_tools_parse_color(const char *text, RPI_Color_t *color) {
    static const struct {
        const char *name;
        RPI_Color_t color;
    } s_color_names[] = {
        {"red", COLOR_RED},
        {"yellow", COLOR_YELLOW},
        {"blue", COLOR_BLUE},
        {"green", COLOR_GREEN},
        {"black", COLOR_BLACK},
        {"lightblue", COLOR_LIGHTBLUE},
    };

    if (color == NULL) return false;

    uint32_t code;
    if (arm_tools_parse_uint(text, (uint32_t)COLOR_RED, (uint32_t)COLOR_LIGHTBLUE, &code)) {
        *color = (RPI_Color_t)code;
        return true;
    }

    for (uint8_t i = 0; i < (uint8_t)(sizeof(s_color_names) / sizeof(s_color_names[0])); i++) {
        if (strcmp(text, s_color_names[i].name) == 0) {
            *color = s_color_names[i].color;
            return true;
        }
    }
    return false;
}

/**
 * @brief 打印 `arm act` 用法/参数范围
 */
static void arm_tools_act_usage(void) {
    logPrintln("Usage: arm act <COMMAND> [args]");
    logPrintln("  arm act grab <color> [pad_slot]                 material area grab -> pad slot (default 1)");
    logPrintln("  arm act process [pad_slot] [slot]               pad pick -> processing place (default 1/1)");
    logPrintln("  arm act storage [pad_slot] [slot] [layer]       pad pick -> storage place (default 1/1/0)");
    logPrintln("  arm act pick-process [slot] [pad_slot]          processing place -> pad slot (default 1/1)");
    logPrintln("  arm act pick-storage [slot] [layer] [pad_slot]  storage place -> pad slot (default 1/0/1)");
    logPrintln("  arm act pose process|storage [slot] [layer]     move to yaw-corrected field place pose only");
    logPrintln("      ('arm yaw <deg>' sets the visual yaw error used by field poses, 'off' clears)");
    logPrintln("color: 1~%d or red/yellow/blue/green/black/lightblue", (int)COLOR_LIGHTBLUE);
    logPrintln(
        "pad_slot: 1~%d(ARM_PAD_SLOT_NUM), slot: 1~%d(ARM_SLOT_NUM), layer: 0/1", ARM_PAD_SLOT_NUM, ARM_SLOT_NUM);
    logPrintln("note: grab waits for RPI color detection (timeout ARM_GRAB_COLOR_TIMEOUT_MS);");
    logPrintln("      all actions use calibrated poses in arm_config.h, same as mission flow.");
    logPrintln("      pick-process/pick-storage are the reverse of process/storage (field -> pad).");
}

/**
 * @brief `arm act grab <color> [pad_slot]`: 原料区抓取指定颜色物料 -> 物料盘指定格位
 * @note 完整任务动作(与 mission 相同): 移到抓取位 -> 等目标颜色稳定 -> 夹爪闭合 -> 抬升 ->
 *       移到物料盘上方 -> 物料盘换格 -> 下降放料 -> 抬起; 需要树莓派颜色识别在线。
 */
static void arm_tools_act_grab(int argc, char *argv[]) {
    RPI_Color_t color;
    uint32_t pad_slot = 1;

    if (argc < 2 || argc > 3) {
        arm_tools_act_usage();
        return;
    }
    if (!arm_tools_parse_color(argv[1], &color)) {
        logPrintln(
            "Invalid color: %s (expect 1~%d or red/yellow/blue/green/black/lightblue)", argv[1], (int)COLOR_LIGHTBLUE);
        return;
    }
    if (argc == 3 && !arm_tools_parse_uint(argv[2], 1, ARM_PAD_SLOT_NUM, &pad_slot)) {
        logPrintln("Invalid pad slot: %s (expect 1~%d)", argv[2], ARM_PAD_SLOT_NUM);
        return;
    }

    logPrintln("Action grab: color %d -> pad slot %u (wait RPI color detection)", (int)color, (unsigned)pad_slot);
    bool ok = Arm_Action_LoadPad(color, (uint8_t)pad_slot);
    logPrintln("Action grab: %s", ok ? "OK" : "FAILED");
}

/**
 * @brief `arm act process [pad_slot] [slot]`: 物料盘取料 -> 粗加工区指定格位
 * @note 完整任务动作: 物料盘换格取料 -> 抬升 -> 移到粗加工区格位 -> 放料 -> 抬升撤离
 */
static void arm_tools_act_process(int argc, char *argv[]) {
    uint32_t pad_slot = 1;
    uint32_t slot = 1;

    if (argc > 3) {
        arm_tools_act_usage();
        return;
    }
    if (argc >= 2 && !arm_tools_parse_uint(argv[1], 1, ARM_PAD_SLOT_NUM, &pad_slot)) {
        logPrintln("Invalid pad slot: %s (expect 1~%d)", argv[1], ARM_PAD_SLOT_NUM);
        return;
    }
    if (argc >= 3 && !arm_tools_parse_uint(argv[2], 1, ARM_SLOT_NUM, &slot)) {
        logPrintln("Invalid processing slot: %s (expect 1~%d)", argv[2], ARM_SLOT_NUM);
        return;
    }

    logPrintln("Action process: pad slot %u -> processing slot %u", (unsigned)pad_slot, (unsigned)slot);
    bool ok = Arm_Action_UnloadToProcessing((uint8_t)pad_slot, (uint8_t)slot, 0);
    logPrintln("Action process: %s", ok ? "OK" : "FAILED");
}

/**
 * @brief `arm act storage [pad_slot] [slot] [layer]`: 物料盘取料 -> 暂存区指定格位/叠层
 * @note 完整任务动作: 物料盘换格取料 -> 抬升 -> 移到暂存区格位(含叠层高度) -> 放料 -> 抬升撤离
 */
static void arm_tools_act_storage(int argc, char *argv[]) {
    uint32_t pad_slot = 1;
    uint32_t slot = 1;
    uint32_t layer = 0;

    if (argc > 4) {
        arm_tools_act_usage();
        return;
    }
    if (argc >= 2 && !arm_tools_parse_uint(argv[1], 1, ARM_PAD_SLOT_NUM, &pad_slot)) {
        logPrintln("Invalid pad slot: %s (expect 1~%d)", argv[1], ARM_PAD_SLOT_NUM);
        return;
    }
    if (argc >= 3 && !arm_tools_parse_uint(argv[2], 1, ARM_SLOT_NUM, &slot)) {
        logPrintln("Invalid storage slot: %s (expect 1~%d)", argv[2], ARM_SLOT_NUM);
        return;
    }
    if (argc >= 4 && !arm_tools_parse_uint(argv[3], 0, 1, &layer)) {
        logPrintln("Invalid storage layer: %s (expect 0/1)", argv[3]);
        return;
    }

    logPrintln(
        "Action storage: pad slot %u -> storage slot %u layer %u", (unsigned)pad_slot, (unsigned)slot, (unsigned)layer);
    bool ok = Arm_Action_UnloadToStorage((uint8_t)pad_slot, (uint8_t)slot, (uint8_t)layer, 0);
    logPrintln("Action storage: %s", ok ? "OK" : "FAILED");
}

/**
 * @brief `arm act pick-process [slot] [pad_slot]`: 粗加工区格位取料 -> 物料盘指定格位
 * @note 完整任务动作(与 `arm act process` 反向): 移到格位取料 -> 抬升撤离 -> 移到物料盘上方
 *       -> 物料盘换格 -> 下降放料 -> 抬起; 与放置动作共用同一套格位位姿计算。
 */
static void arm_tools_act_pick_process(int argc, char *argv[]) {
    uint32_t slot = 1;
    uint32_t pad_slot = 1;

    if (argc > 3) {
        arm_tools_act_usage();
        return;
    }
    if (argc >= 2 && !arm_tools_parse_uint(argv[1], 1, ARM_SLOT_NUM, &slot)) {
        logPrintln("Invalid processing slot: %s (expect 1~%d)", argv[1], ARM_SLOT_NUM);
        return;
    }
    if (argc >= 3 && !arm_tools_parse_uint(argv[2], 1, ARM_PAD_SLOT_NUM, &pad_slot)) {
        logPrintln("Invalid pad slot: %s (expect 1~%d)", argv[2], ARM_PAD_SLOT_NUM);
        return;
    }

    logPrintln("Action pick-process: processing slot %u -> pad slot %u", (unsigned)slot, (unsigned)pad_slot);
    bool ok = Arm_Action_LoadPadFromProcessing((uint8_t)slot, (uint8_t)pad_slot);
    logPrintln("Action pick-process: %s", ok ? "OK" : "FAILED");
}

/**
 * @brief `arm act pick-storage [slot] [layer] [pad_slot]`: 暂存区格位/叠层取料 -> 物料盘指定格位
 * @note 完整任务动作(与 `arm act storage` 反向): 移到格位(含叠层高度)取料 -> 抬升撤离
 *       -> 移到物料盘上方 -> 换格 -> 下降放料 -> 抬起。
 */
static void arm_tools_act_pick_storage(int argc, char *argv[]) {
    uint32_t slot = 1;
    uint32_t layer = 0;
    uint32_t pad_slot = 1;

    if (argc > 4) {
        arm_tools_act_usage();
        return;
    }
    if (argc >= 2 && !arm_tools_parse_uint(argv[1], 1, ARM_SLOT_NUM, &slot)) {
        logPrintln("Invalid storage slot: %s (expect 1~%d)", argv[1], ARM_SLOT_NUM);
        return;
    }
    if (argc >= 3 && !arm_tools_parse_uint(argv[2], 0, 1, &layer)) {
        logPrintln("Invalid storage layer: %s (expect 0/1)", argv[2]);
        return;
    }
    if (argc >= 4 && !arm_tools_parse_uint(argv[3], 1, ARM_PAD_SLOT_NUM, &pad_slot)) {
        logPrintln("Invalid pad slot: %s (expect 1~%d)", argv[3], ARM_PAD_SLOT_NUM);
        return;
    }

    logPrintln("Action pick-storage: storage slot %u layer %u -> pad slot %u",
               (unsigned)slot,
               (unsigned)layer,
               (unsigned)pad_slot);
    bool ok = Arm_Action_LoadPadFromStorage((uint8_t)slot, (uint8_t)layer, (uint8_t)pad_slot);
    logPrintln("Action pick-storage: %s", ok ? "OK" : "FAILED");
}

/**
 * @brief `arm act pose <process|storage> [slot] [layer]`: 只移动到(叠加 yaw 补偿后的)场地放置位
 * @note 调试用: 不做夹取/放料, 只走位并打印目标/实际极坐标, 便于核对视觉 yaw 补偿后的基座角度。
 *       配合 `arm yaw <deg>` 可脱离整任务流程验证补偿方向与幅度。
 */
static void arm_tools_act_pose(int argc, char *argv[]) {
    if (argc < 2 || argc > 4) {
        arm_tools_act_usage();
        return;
    }

    ArmPlaceArea_t area;
    uint8_t max_argc;
    if (strcmp(argv[1], "process") == 0) {
        area = ARM_PLACE_AREA_PROCESSING;
        max_argc = 3;
    } else if (strcmp(argv[1], "storage") == 0) {
        area = ARM_PLACE_AREA_STORAGE;
        max_argc = 4;
    } else {
        logPrintln("Unknown area: %s (expect process|storage)", argv[1]);
        arm_tools_act_usage();
        return;
    }
    if (argc > max_argc) {
        arm_tools_act_usage();
        return;
    }

    uint32_t slot = 1;
    uint32_t layer = 0;
    if (argc >= 3 && !arm_tools_parse_uint(argv[2], 1, ARM_SLOT_NUM, &slot)) {
        logPrintln("Invalid slot: %s (expect 1~%d)", argv[2], ARM_SLOT_NUM);
        return;
    }
    if (argc >= 4 && !arm_tools_parse_uint(argv[3], 0, 1, &layer)) {
        logPrintln("Invalid layer: %s (expect 0/1)", argv[3]);
        return;
    }

    ArmPose_t target;
    if (!Arm_Action_CalcFieldPose(area, (uint8_t)slot, (uint8_t)layer, &target)) {
        logPrintln("Calc field pose failed");
        return;
    }

    logPrintln("Action pose: %s slot %u layer %u -> target (extend %.2f, angle %.2f, lift %.2f), yaw corr %.2f deg",
               argv[1],
               (unsigned)slot,
               (unsigned)layer,
               (double)target.extend_mm,
               (double)target.base_angle_deg,
               (double)target.lift_mm,
               (double)Arm_Action_GetYawCorrection());
    Arm_SetMotionProfile(NULL); /* 调试移动用默认速度/加速度 */
    if (!Arm_MovePolarToAndWait(target.extend_mm, target.base_angle_deg, target.lift_mm, ARM_MOVE_TIMEOUT_MS)) {
        logPrintln("Action pose: FAILED");
        return;
    }

    ArmPolarPose_t actual;
    if (Arm_GetPolarPose(&actual)) {
        logPrintln("Action pose OK: actual (extend %.2f, angle %.2f, lift %.2f)",
                   (double)actual.extend_mm,
                   (double)actual.base_angle_deg,
                   (double)actual.lift_mm);
    } else {
        logPrintln("Action pose OK (actual pose unavailable)");
    }
}

/**
 * @brief `arm yaw [deg|off]`: 查看/设置视觉 yaw 误差的机械臂补偿角(标定补偿联调用)
 * @note 参数按"rpi cal 给出的视觉 yaw 误差"理解, 内部按 ARM_YAW_CORRECTION_SIGN 换算并限幅;
 *       设置后 `arm act ...` 的场地固定工位基座旋转会叠加该角度; `off`/`0` 清除。
 */
static void arm_tools_yaw(int argc, char *argv[]) {
    if (argc > 2) {
        logPrintln("Usage: arm yaw [deg|off]");
        return;
    }

    if (argc == 2) {
        if (strcmp(argv[1], "off") == 0) {
            Arm_Action_SetYawCorrection(0.0f);
        } else {
            char *end = NULL;
            double value = strtod(argv[1], &end);
            if (end == argv[1] || *end != '\0' || value < -90.0 || value > 90.0) {
                logPrintln("Invalid yaw error: %s (expect -90~90 deg or off)", argv[1]);
                return;
            }
            Arm_Action_SetYawCorrection((float)value);
        }
    }

    logPrintln("yaw correction: applied %.2f deg (enable=%d, sign=%.1f, max=%.1f)",
               (double)Arm_Action_GetYawCorrection(),
               ARM_YAW_CORRECTION_ENABLE,
               (double)ARM_YAW_CORRECTION_SIGN,
               (double)ARM_YAW_CORRECTION_MAX_DEG);
    logPrintln("usage: arm yaw <deg>  set visual yaw error (field poses base angle += corr)");
    logPrintln("       arm yaw off      clear correction");
}

/**
 * @brief `arm key [extend_step_mm] [angle_step_deg] [lift_step_mm]`: 键盘点动控制机械臂
 * @note 按键: W/S=伸展伸长/收回, A/D=基座左转/右转, Q/E=升降/下降,
 *       Z/X=夹爪张开/闭合(内部等角度回读确认到位, 不参与三轴目标累计), ^C=退出。
 *       每按一次键下发一次"当前目标 + 步长"的单轴绝对位置命令(非阻塞), 速度/加速度沿用各轴默认参数;
 *       软限位由单轴接口检查(超限只提示不运动); 终端长按自动重复按键, 间隔由 ARM_KEY_REPEAT_MS 限制。
 *       步长缺省值在 arm_config.h 的"键盘点动(arm key)"区配置。
 */
static void arm_tools_key(int argc, char *argv[]) {
    Shell *shell = shellGetCurrent();
    if (shell == NULL) {
        return;
    }
    if (argc > 4) {
        logPrintln("Usage: arm key [extend_step_mm] [angle_step_deg] [lift_step_mm]");
        return;
    }

    float extend_step = ARM_KEY_EXTEND_STEP_MM;
    float angle_step = ARM_KEY_ANGLE_STEP_DEG;
    float lift_step = ARM_KEY_LIFT_STEP_MM;
    if (argc >= 2) extend_step = (float)atof(argv[1]);
    if (argc >= 3) angle_step = (float)atof(argv[2]);
    if (argc >= 4) lift_step = (float)atof(argv[3]);
    if (extend_step <= 0.0f || angle_step <= 0.0f || lift_step <= 0.0f) {
        logPrintln("Usage: arm key [extend_step_mm] [angle_step_deg] [lift_step_mm]");
        return;
    }

    /* 以当前实测极坐标为起点(未回零时仍可点动, 但绝对坐标不可信) */
    ArmPolarPose_t pose;
    if (!Arm_GetPolarPose(&pose)) {
        logPrintln("Arm pose unavailable (check motors/homing, run 'arm pos' first)");
        return;
    }
    if (!Arm_IsHomeDone()) {
        logPrintln("[arm] warning: homing not done this boot, absolute position may be wrong");
    }

    float extend_mm = pose.extend_mm;
    float angle_deg = pose.base_angle_deg;
    float lift_mm = pose.lift_mm;

    logPrintln("Arm key control: W/S=extend, A/D=rotate, Q/E=lift, Z/X=gripper, ^C=exit");
    logPrintln("start: extend %.1fmm, angle %.1fdeg, lift %.1fmm (step %.1fmm / %.1fdeg / %.1fmm)",
               (double)extend_mm,
               (double)angle_deg,
               (double)lift_mm,
               (double)extend_step,
               (double)angle_step,
               (double)lift_step);

    /* 接管串口输入: 置位 APP_NEED_USART 后 shell 任务不再消费按键, 由本命令读取 */
    osEventFlagsSet(System_StatusHandle, APP_NEED_USART);

    uint32_t last_step_tick = osKernelGetTickCount();
    char key;
    for (;;) {
        uint8_t ret = shell->read(&key, 1);
        if (ret > 0) {
            if (key == 0x03) { // ^C 退出
                break;
            }
            if ((osKernelGetTickCount() - last_step_tick) < ARM_KEY_REPEAT_MS) {
                continue; // 终端自动重复过快时忽略, 防止一按就冲出去
            }
            last_step_tick = osKernelGetTickCount();

            bool moved = true;
            switch (key) {
            case 'w':
            case 'W': // 伸展伸长
                extend_mm += extend_step;
                if (!Arm_ExtendTo(extend_mm, ARM_EXTEND_VEL, ARM_EXTEND_ACC)) {
                    extend_mm -= extend_step;
                    moved = false;
                }
                break;
            case 's':
            case 'S': // 伸展收回
                extend_mm -= extend_step;
                if (!Arm_ExtendTo(extend_mm, ARM_EXTEND_VEL, ARM_EXTEND_ACC)) {
                    extend_mm += extend_step;
                    moved = false;
                }
                break;
            case 'a':
            case 'A': // 基座左转(角度增大方向)
                angle_deg += angle_step;
                if (!Arm_BaseRotateAbs(
                        (int32_t)(angle_deg + (angle_deg >= 0.0f ? 0.5f : -0.5f)), ARM_BASE_VEL, ARM_BASE_ACC)) {
                    angle_deg -= angle_step;
                    moved = false;
                }
                break;
            case 'd':
            case 'D': // 基座右转(角度减小方向)
                angle_deg -= angle_step;
                if (!Arm_BaseRotateAbs(
                        (int32_t)(angle_deg + (angle_deg >= 0.0f ? 0.5f : -0.5f)), ARM_BASE_VEL, ARM_BASE_ACC)) {
                    angle_deg += angle_step;
                    moved = false;
                }
                break;
            case 'q':
            case 'Q': // 上升
                lift_mm += lift_step;
                if (!Arm_LiftTo(lift_mm, ARM_LIFT_VEL, ARM_LIFT_ACC)) {
                    lift_mm -= lift_step;
                    moved = false;
                }
                break;
            case 'e':
            case 'E': // 下降
                lift_mm -= lift_step;
                if (!Arm_LiftTo(lift_mm, ARM_LIFT_VEL, ARM_LIFT_ACC)) {
                    lift_mm += lift_step;
                    moved = false;
                }
                break;
            case 'z':
            case 'Z': // 夹爪张开(不参与三轴目标累计)
                moved = false;
                logPrintln("gripper open: %s", Arm_GripperSetAngle(ARM_GRIPPER_OPEN_DEG) ? "OK" : "FAILED");
                break;
            case 'x':
            case 'X': // 夹爪闭合
                moved = false;
                logPrintln("gripper close: %s", Arm_GripperSetAngle(ARM_GRIPPER_CLOSE_DEG) ? "OK" : "FAILED");
                break;
            default:
                moved = false;
                break;
            }

            if (moved) {
                logPrintln(
                    "extend %.1fmm, angle %.1fdeg, lift %.1fmm", (double)extend_mm, (double)angle_deg, (double)lift_mm);
            } else if (key == 'w' || key == 'W' || key == 's' || key == 'S' || key == 'a' || key == 'A' || key == 'd' ||
                       key == 'D' || key == 'q' || key == 'Q' || key == 'e' || key == 'E') {
                logPrintln("move rejected (out of range/offline): extend %.1fmm, angle %.1fdeg, lift %.1fmm",
                           (double)extend_mm,
                           (double)angle_deg,
                           (double)lift_mm);
            }
        }
        osDelay(10);
    }

    osEventFlagsClear(System_StatusHandle, APP_NEED_USART);
    logPrintln("Arm key control exit (current target: extend %.1fmm, angle %.1fdeg, lift %.1fmm)",
               (double)extend_mm,
               (double)angle_deg,
               (double)lift_mm);
}

/**
 * @brief `arm act ...`: 完整任务动作调试(抓取/取放到物料盘、粗加工区、暂存区)
 * @note 与 mission 任务流程调用同一套 Arm_Action_* 接口(含夹爪/物料盘/颜色识别/到位等待),
 *       用于整段动作联调; 目标位姿需先在 arm_config.h 标定好。
 *       正向: grab(原料区->物料盘)、process/storage(物料盘->场地);
 *       反向: pick-process/pick-storage(场地->物料盘);
 *       pose: 只走到(叠加 yaw 补偿后的)场地放置位, 核对补偿用。
 */
static void arm_tools_act(int argc, char *argv[]) {
    if (argc < 2) {
        arm_tools_act_usage();
        return;
    }

    /* 绝对位姿动作要求已回零: 未回零时提示(手动 arm zero 的流程由用户自行确认) */
    if (!Arm_IsHomeDone()) {
        logPrintln("[arm] warning: homing not done this boot, run 'arm init' first if pose is not manually zeroed");
    }

    /* 视觉 yaw 补偿生效时提示, 便于联调时确认动作确实叠加了补偿 */
    float yaw_corr = Arm_Action_GetYawCorrection();
    if (yaw_corr > 0.01f || yaw_corr < -0.01f) {
        logPrintln("[arm] yaw correction active: %.2f deg (field poses shifted, 'arm yaw off' to clear)",
                   (double)yaw_corr);
    }

    if (strcmp(argv[1], "grab") == 0) {
        arm_tools_act_grab(argc - 1, &argv[1]);
    } else if (strcmp(argv[1], "process") == 0) {
        arm_tools_act_process(argc - 1, &argv[1]);
    } else if (strcmp(argv[1], "storage") == 0) {
        arm_tools_act_storage(argc - 1, &argv[1]);
    } else if (strcmp(argv[1], "pick-process") == 0) {
        arm_tools_act_pick_process(argc - 1, &argv[1]);
    } else if (strcmp(argv[1], "pick-storage") == 0) {
        arm_tools_act_pick_storage(argc - 1, &argv[1]);
    } else if (strcmp(argv[1], "pose") == 0) {
        arm_tools_act_pose(argc - 1, &argv[1]);
    } else {
        logPrintln("Unknown action: %s", argv[1]);
        arm_tools_act_usage();
    }
}

ShellCommand ArmToolsGroup[] = {
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, init, arm_tools_init, Initialize Arm Tools),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, home, arm_tools_home, Home A Single Arm Axis),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, ctrl, arm_tools_ctrl, Control Arm to Target Position),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, pol, arm_tools_polar, Control Arm by Polar Coord),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, pos, arm_tools_pos, Read Arm Pose),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, en, arm_tools_enable, Enable / Disable Motors),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, zero, arm_tools_zero, Set Current Position As Zero),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, gri, arm_tools_gripper, Control Gripper),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, pad, arm_tools_pad, Move Material Pad Slot),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, flip, arm_tools_flip, Control Camera Flip Servo),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, preset, arm_tools_preset, Go to Preset Pose),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, presets, arm_tools_presets, List Preset Poses),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, act, arm_tools_act, Run Full Arm Task Action),
    SHELL_CMD_GROUP_ITEM(
        SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN, yaw, arm_tools_yaw, Set / Clear Visual Yaw Correction),
    SHELL_CMD_GROUP_ITEM(SHELL_TYPE_CMD_MAIN | SHELL_CMD_DISABLE_RETURN,
                         key,
                         arm_tools_key,
                         Keyboard Jog Control(W / S / A / D / Q / E)),
    SHELL_CMD_GROUP_END()};
SHELL_EXPORT_CMD_GROUP(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                       arm,
                       ArmToolsGroup,
                       Arm Tools);

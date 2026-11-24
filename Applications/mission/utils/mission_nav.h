/**
 * @file mission_nav.h
 * @author T-Car Team
 * @brief 任务导航封装
 *
 * 屏蔽"直接导航(Nav) / 路径规划(Planner)"两种实现差异，并在其上提供
 * "任务路线（固定顺序的若干导航段）"的声明式执行接口：
 *   任务流程只描述"经过哪些点、用哪种导航方式、要不要到位精修"，
 *   由本模块统一做：启动导航 -> 等待到位 -> 失败重试 -> 中止检查 -> 到位精修。
 *
 * 这样做的原因：规划器依赖实时定位，定位抖动会改变 A* 平局/旋转点选择，
 * 长距离两点之间可能走出不同路线。把任务拆成固定途经点序列（MissionRoute_t）
 * 后，每段都短且方向明确，路线与旋转点即可复现；调整路线只改路线表数据。
 *
 * 注意：任务级二次标定链路（CalibrateAt）除树莓派视觉命令外，还包含"机械臂预备位姿
 * （基座旋转/伸展/抬升，按目标区域配置）+ 相机翻转"（依赖 arm_action 的
 * Arm_Action_CalibrationReady / Arm_Action_CalibrationFinish，相机装在机械臂末端时必需）；
 * 该链路仅在 MISSION_CAL_ENABLE=1（即 MISSION_DEBUG_MODE=0）时编译。
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#ifndef __MISSION_NAV_H__
#define __MISSION_NAV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 任务级二次标定类型
 */
typedef enum {
    MISSION_CAL_NONE = 0, // 不标定
    MISSION_CAL_DISK, // 校准圆盘(原料区)
    MISSION_CAL_RING_FIRST, // 校准圆环-第一次(粗加工区)
    MISSION_CAL_RING_SECOND, // 校准圆环-第二次(保留)
    MISSION_CAL_STORAGE, // 暂存区标定(树莓派命令 0xE1, 应答格式同圆环)
} MissionCal_t;

/**
 * @brief 单段导航方式
 *
 * 到位语义以各导航实现为准：
 *   - 规划器（Planner_GoTo_*）返回 true = 路径执行完成且终点精度校验通过
 *     （位置约 ±1.2cm、朝向约 ±2.5°；终点旋转不安全时接受更大 yaw 误差）；
 *   - 直连（Nav_GoToDirect）返回 true = 导航层进入 NAV_STATE_COMPLETE。
 */
typedef enum {
    MISSION_NAV_MODE_AUTO = 0, // 自动：规划器启用时用规划器，否则直连（默认）
    MISSION_NAV_MODE_PLANNER, // 优先规划器：按地图逐航点绕障执行（PLANNER_ENABLED=0 时退化为直连）
    MISSION_NAV_MODE_DIRECT, // 强制直连：朝目标点直线行驶，不做绕障
    MISSION_NAV_MODE_ROTATE, // 原地旋转
    /**
     * 不停车通过点（过弯）：向该点直连导航，进入触发距离（MISSION_PASS_TRIGGER_CM）后
     * 直接切换下一段目标（Nav_Retarget，保留速度状态），不在该点停车；
     * 下一段若是普通段，则按普通段语义继续到点 + 标定/精修。
     * 注意：PASS 段后面必须还有一段（不能是路线最后一段），且不能紧跟 ROTATE 段；
     * 单独作为一段执行时会退化为普通到点。
     */
    MISSION_NAV_MODE_PASS,
} MissionNavMode_t;

/**
 * @brief 任务路线中的一段导航
 *
 * 目标点二选一：
 *   - target_name != NULL：按点名导航（点位来自 nav_map_data.c，已换算到车体参考点）；
 *   - target_name == NULL：使用 x_cm / y_cm / yaw_deg 的临时点（与任务点同坐标约定，
 *     本模块会做同样的车体参考点换算）。临时点通常只用 DIRECT / PASS 方式。
 *   - mode == MISSION_NAV_MODE_ROTATE：原地旋转段（不算平移），只用 yaw_deg，
 *     位置锁"段开始时实测的导航位姿"，不做偏移换算、不做到位精修。
 *   - mode == MISSION_NAV_MODE_PASS：不停车通过点（过弯），到触发距离即换下一段目标，
 *     不停车；过弯后的朝向由下一段目标 yaw 决定（边走边转）。
 *
 * 到位后处理（按优先级）：
 *   1) cal_type != MISSION_CAL_NONE 且 target_name != NULL：
 *      调用树莓派视觉二次标定（RPI_Cal），拿到"固定偏移 + 视觉误差"的相对修正量后
 *      用底盘位置模式精修并等待到位（偏移为 0 是合法值）；标定失败按任务失败处理。
 *   2) 否则 align = true 时做一次"只修位置、不转朝向"的精修（best-effort，失败只告警）。
 *      终点朝向由规划器两段式对齐负责，任务层不强制在原地转体，避免旋转不安全时扫障。
 */
typedef struct {
    const char *target_name; // 目标点名；NULL 表示使用下面的临时点坐标
    float x_cm; // 临时点 X (cm)
    float y_cm; // 临时点 Y (cm)
    float yaw_deg; // 临时点朝向 (deg)
    MissionNavMode_t mode; // 导航方式
    bool align; // 到位后是否做位置精修（未启用标定时生效）
    MissionCal_t cal_type; // 到位后树莓派二次标定类型（MISSION_CAL_NONE = 不标定）
    int cal_offset_x_mm; // 二次标定的机体系 X 固定偏移(mm, +前)，cal_type != MISSION_CAL_NONE 时使用
    int cal_offset_y_mm; // 二次标定的机体系 Y 固定偏移(mm, +左)，同上
} MissionLeg_t;

/**
 * @brief 任务路线：固定顺序的若干导航段
 * @note  count == 0 表示空路线（执行时直接返回成功），用于不同启动区/轮次的差异路线
 */
typedef struct {
    const MissionLeg_t *legs; // 导航段数组
    uint8_t count; // 导航段数量
} MissionRoute_t;

/**
 * @brief 点名导航并等待到位（需要时通过树莓派做二次标定）
 * @param target_name    目标点名称（须与地图点名称一致）
 * @param cal_type       标定类型；MISSION_CAL_NONE 表示不做二次标定
 * @param cal_offset_x_mm 标定后的机体系 X 固定偏移(mm, +前)；0 是合法值
 * @param cal_offset_y_mm 标定后的机体系 Y 固定偏移(mm, +左)
 * @return true 已到位
 * @return false 点名不存在 / 规划失败 / 导航失败 / 任务被中止 / 标定失败
 */
bool Mission_NavGoTo(const char *target_name, MissionCal_t cal_type, int cal_offset_x_mm, int cal_offset_y_mm);

/**
 * @brief 按指定方式点名导航并等待到位（失败自动重试，支持任务中止）
 * @param target_name 目标点名称
 * @param mode        导航方式
 * @return true 已到位；false 失败/中止
 */
bool Mission_NavGoToEx(const char *target_name, MissionNavMode_t mode);

/**
 * @brief 执行一段任务路线导航（含重试、中止检查、到位后二次标定/位置精修）
 * @param leg 导航段
 * @return true 已到位（未启用标定时精修失败不算失败）；false 导航/标定失败或任务中止
 */
bool Mission_NavGoToLeg(const MissionLeg_t *leg);

/**
 * @brief 按顺序执行一条任务路线
 * @param route 路线；NULL 或 count==0 视为空路线，直接成功
 * @param tag   日志标签（如 "grab"/"storage"），便于现场定位问题
 * @return true 全部到位；false 任一段失败/任务中止
 */
bool Mission_NavGoToRoute(const MissionRoute_t *route, const char *tag);
/**
 * @brief 停止导航输出
 */
void Mission_NavStop(void);

/**
 * @brief 请求中止正在阻塞的导航/规划调用
 * @note  供任务停止接口调用；只置中止请求，不直接操作电机，
 *        避免在非任务上下文中操作导航状态
 */
void Mission_NavAbort(void);

/**
 * @brief 等待导航到目标点（阻塞调用）
 * @return true 到位
 * @return false 导航失败或任务被中止（已顺带停止导航）
 */
bool Wait_Arrive(void);

/**
 * @brief 启动一次"车体原地旋转"（非阻塞，位置锁当前实测位姿，只改朝向）
 * @param yaw_deg 目标朝向（°）
 * @return true=旋转已下发；false=取不到当前位姿/下发失败
 * @note  供任务层在"每段导航出发前的准备"期间使用：路线首段是原地旋转段时，把这段旋转提前到
 *        物料盘归位期间并行执行，与 Mission_NavWaitRotateInPlace 配对。
 *        位置直接取调用瞬间的实测导航位姿，不做车体参考点换算（与路线表的 ROTATE 段同一语义），
 *        所以无论 CHASSIS_BODY_REF_OFFSET 是否为 0 都只转体、不平移。
 */
bool Mission_NavStartRotateInPlace(float yaw_deg);

/**
 * @brief 等待 Mission_NavStartRotateInPlace 启动的原地旋转到位
 * @return true=已到位（未启动过也返回 true）；false=导航失败/超时/任务被中止
 * @note  失败时调用方应回退为按原路线执行（首段的旋转由路线自己再完成一次），不要直接判任务失败。
 */
bool Mission_NavWaitRotateInPlace(void);

#ifdef __cplusplus
}
#endif

#endif /* __MISSION_NAV_H__ */

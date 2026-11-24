/**
 * @file mission.h
 * @author T-Car Team
 * @brief 总任务模块公共接口
 * 可调参数见 mission_config.h / Applications/mission/README.md。
 */

#ifndef __MISSION_H__
#define __MISSION_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>

#define MISSION_SHELL 1

/**
 * @brief 设置总任务运行状态
 * @note  由按键/调试命令等外部入口调用；停止时会置任务中止标志并中止阻塞中的导航/规划
 * @param running 运行状态
 */
void mission_set_running(bool running);

/**
 * @brief 查询总任务是否正在运行
 * @note  供任务子模块（如 mission_arm 的机械臂动作编排）检查任务是否已被请求停止
 * @return true=正在运行; false=未运行或已被请求停止
 */
bool mission_is_running(void);

/**
 * @brief 查询任务运行记时（从 run 到 stop/完成/失败）
 * @param elapsed_ms 输出: 运行时长(ms), 可为 NULL
 * @return true=任务正在运行(计时中); false=任务未运行(此时 elapsed_ms 为最近一次运行的时长)
 * @note  计时起点: mission_set_running(true)（按键启动 / `mission run`）；
 *        计时终点: 任务完成、任务失败或 mission_set_running(false)（`mission stop`）。
 *        任务完成后时长保留，便于用 `mission time` 查看；再次 run 会重新从 0 计时。
 */
bool mission_get_run_time(uint32_t *elapsed_ms);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MISSION_H__ */

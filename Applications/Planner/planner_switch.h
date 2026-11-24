/**
 * @file planner_switch.h
 * @brief 规划器总开关
 *
 * @details 开关单独成文件，是为了让不需要规划器内部配置的模块（如 nav 地图点位数据）
 *          也能读到同一个开关，避免 planner_config.h 里的大块地图/障碍物配置被一起包含。
 */

#ifndef __PLANNER_SWITCH_H__
#define __PLANNER_SWITCH_H__

/**
 * @brief 是否启用路径规划器
 * @note  1 = 使用规划器导航（Planner_GoTo_*）；
 *        0 = 不使用规划器，直接导航（Nav_GoTo_*），地图点位自动切换到不使用规划器版
 */
#ifndef PLANNER_ENABLED
#define PLANNER_ENABLED 1
#endif

#endif /* __PLANNER_SWITCH_H__ */

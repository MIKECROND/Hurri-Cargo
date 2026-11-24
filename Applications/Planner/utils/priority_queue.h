/**
 * @file priority_queue.h
 * @author T-Car Team
 * @brief 声明供 A* 路径搜索使用的优先队列数据结构与对外接口。
 * 可调参数见 planner_config.h / Applications/Planner/README.md。
 */

/**
 * @brief 优先队列实现
 * @details 包含最小堆的实现，用于 A* 算法的路径搜索
 */

#ifndef STM32_PLANNER_PC_TEST_PRIORITY_QUEUE_H
#define STM32_PLANNER_PC_TEST_PRIORITY_QUEUE_H
#include <stdbool.h>
#include <stdint.h>

#include "planner_config.h"

#if PLANNER_ENABLED

typedef struct {
    uint32_t *data;
    int size;
    int capacity;
    bool overflowed;
} PriorityQueue;

void pq_init(PriorityQueue *pq);
bool pq_push(PriorityQueue *pq, uint16_t idx, uint16_t f);
bool pq_pop(PriorityQueue *pq, uint16_t *idx, uint16_t *f);

#endif

#endif //STM32_PLANNER_PC_TEST_PRIORITY_QUEUE_H

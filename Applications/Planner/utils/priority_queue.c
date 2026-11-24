/**
 * @file priority_queue.c
 * @author T-Car Team
 * @brief 实现基于最小堆的优先队列，供 A* 路径搜索使用。
 * 可调参数见 planner_config.h / Applications/Planner/README.md。
 */

/**
 * @brief 优先队列实现
 * @details 包含最小堆的实现，用于 A* 算法的路径搜索
 */

#include "priority_queue.h"
#include "planner_config.h"

#if PLANNER_ENABLED
#if PLANNER_PLATFORM == PLATFORM_MCU
/**
 * @brief 初始化优先队列
 * @details pq 优先队列指针
 */
void pq_init(PriorityQueue *pq) {
    // 将指针指向共享内存池的阶段 1 区域
    pq->data = g_mem_pool.pq_data;
    pq->size = 0;
    pq->capacity = MAX_PQ_SIZE;
    pq->overflowed = false;
}

// ---------------- 最小堆实现 (极限优化版) ----------------
/**
 * @brief 向优先队列插入一个元素
 * @details pq 优先队列指针
 * @param idx 元素索引
 * @param f 元素代价
 * @return 是否插入成功
 */
bool pq_push(PriorityQueue *pq, uint16_t idx, uint16_t f) {
    if (pq->size >= pq->capacity) {
        pq->overflowed = true;
        return false;
    }

    // 【核心优化 1】：打包数据。f 放高 16 位，idx 放低 16 位
    uint32_t element = ((uint32_t)f << 16) | idx;

    int i = pq->size++;
    while (i > 0) {
        // 【核心优化 2】：位移代替除法 (单周期指令)
        int p = (i - 1) >> 1;

        uint32_t parent_val = pq->data[p];
        // 这里的 <= 直接比较了 32 位数据。
        // 因为 f 在高 16 位，所以优先比较 f；
        // 如果 f 相同，会自动比较低 16 位的 idx 作为 Tie-breaker。
        if (parent_val <= element) break;

        pq->data[i] = parent_val;
        i = p;
    }
    pq->data[i] = element;
    return true;
}
/**
 * @brief 从优先队列弹出代价代价最小的元素
 * @details pq 优先队列指针
 * @param idx 弹出元素索引指针
 * @param f 弹出元素代价指针
 * @return 是否弹出成功
 */
bool pq_pop(PriorityQueue *pq, uint16_t *idx, uint16_t *f) {
    if (pq->size == 0) return false;

    // 【核心优化 3】：解包数据
    uint32_t top = pq->data[0];
    *idx = (uint16_t)(top & 0xFFFF);
    *f = (uint16_t)(top >> 16);

    pq->size--;
    if (pq->size == 0) return true;

    uint32_t temp = pq->data[pq->size];
    int i = 0;
    int size = pq->size;

    // 移除多余的容量计算，精简循环内部逻辑
    while (1) {
        // 【核心优化 4】：位移代替乘法
        int left = (i << 1) + 1;
        if (left >= size) break; // 没有左孩子，说明是叶子节点

        int right = left + 1;
        int min_child = left;
        uint32_t min_val = pq->data[left];

        // 寻找左右孩子中较小的一个（同样利用 32 位全比较）
        if (right < size) {
            uint32_t right_val = pq->data[right];
            if (right_val < min_val) {
                min_child = right;
                min_val = right_val;
            }
        }

        if (temp <= min_val) break;

        pq->data[i] = min_val;
        i = min_child;
    }
    pq->data[i] = temp;
    return true;
}
#endif // PLANNER_PLATFORM

#endif

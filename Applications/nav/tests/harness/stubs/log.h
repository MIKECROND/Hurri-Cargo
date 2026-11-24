/**
 * @file log.h
 * @brief 主机端仿真用 log 最小桩
 *
 * 真实 log.h 依赖 letter-shell / 串口 DMA，主机端无法链接；这里只保留 nav 控制代码
 * 使用的 logInfo/logWarning/logError/logDebug/logVerbose 宏，统一转到 Sim_LogWrite()
 * （实现在 sim_platform.c，可按 -v 打开或静默）。
 */

#ifndef __LOG_H__
#define __LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

void Sim_LogWrite(char level, const char *fmt, ...);

#define logInfo(...) Sim_LogWrite('I', __VA_ARGS__)
#define logWarning(...) Sim_LogWrite('W', __VA_ARGS__)
#define logError(...) Sim_LogWrite('E', __VA_ARGS__)
#define logDebug(...) Sim_LogWrite('D', __VA_ARGS__)
#define logVerbose(...) Sim_LogWrite('V', __VA_ARGS__)
#define logPrintln(...) Sim_LogWrite('P', __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H__ */

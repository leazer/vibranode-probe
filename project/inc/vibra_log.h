/**
 * @file vibra_log.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef VIBRA_LOG_H
#define VIBRA_LOG_H

#define VLOG_LEVEL_NONE        0
#define VLOG_LEVEL_ERROR       1
#define VLOG_LEVEL_WARN        2
#define VLOG_LEVEL_INFO        3
#define VLOG_LEVEL_DEBUG       4
#define VLOG_LEVEL_VERBOSE     5

#include "vibra_log_config.h"

#if VLOG_ENABLE

#include "SEGGER_RTT.h"

#if VLOG_TIMESTAMP_ENABLE
#include "wk_system.h"
#endif

#ifndef LOG_TAG
#define LOG_TAG "APP"
#endif

#define LOG_INIT() SEGGER_RTT_Init()

#if VLOG_TIMESTAMP_ENABLE
#define VLOG_WRITE(level, format, ...)                                          \
  ((void)SEGGER_RTT_printf(VLOG_RTT_BUFFER_INDEX,                               \
                           "[%010u][" level "][" LOG_TAG "] " format "\r\n", \
                           (unsigned int)wk_timebase_get(), ##__VA_ARGS__))
#else
#define VLOG_WRITE(level, format, ...)                                          \
  ((void)SEGGER_RTT_printf(VLOG_RTT_BUFFER_INDEX,                               \
                           "[" level "][" LOG_TAG "] " format "\r\n",       \
                           ##__VA_ARGS__))
#endif

#else

#define LOG_INIT() ((void)0)

#endif

#if VLOG_ENABLE && (VLOG_LEVEL >= VLOG_LEVEL_ERROR)
#define LOG_E(format, ...) VLOG_WRITE("E", format, ##__VA_ARGS__)
#else
#define LOG_E(...) ((void)0)
#endif

#if VLOG_ENABLE && (VLOG_LEVEL >= VLOG_LEVEL_WARN)
#define LOG_W(format, ...) VLOG_WRITE("W", format, ##__VA_ARGS__)
#else
#define LOG_W(...) ((void)0)
#endif

#if VLOG_ENABLE && (VLOG_LEVEL >= VLOG_LEVEL_INFO)
#define LOG_I(format, ...) VLOG_WRITE("I", format, ##__VA_ARGS__)
#else
#define LOG_I(...) ((void)0)
#endif

#if VLOG_ENABLE && (VLOG_LEVEL >= VLOG_LEVEL_DEBUG)
#define LOG_D(format, ...) VLOG_WRITE("D", format, ##__VA_ARGS__)
#else
#define LOG_D(...) ((void)0)
#endif

#if VLOG_ENABLE && (VLOG_LEVEL >= VLOG_LEVEL_VERBOSE)
#define LOG_V(format, ...) VLOG_WRITE("V", format, ##__VA_ARGS__)
#else
#define LOG_V(...) ((void)0)
#endif

#endif

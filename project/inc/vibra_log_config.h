/**
 * @file vibra_log_config.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef VIBRA_LOG_CONFIG_H
#define VIBRA_LOG_CONFIG_H

/* Master switch for all VibraNode log output. */
#ifndef VLOG_ENABLE
#define VLOG_ENABLE             1
#endif

/* Compile-time maximum verbosity. */
#ifndef VLOG_LEVEL
#define VLOG_LEVEL              VLOG_LEVEL_DEBUG
#endif

/* Prefix each log line with the millisecond uptime. */
#ifndef VLOG_TIMESTAMP_ENABLE
#define VLOG_TIMESTAMP_ENABLE   1
#endif

/* SEGGER RTT up-buffer used by the logging macros. */
#ifndef VLOG_RTT_BUFFER_INDEX
#define VLOG_RTT_BUFFER_INDEX   0U
#endif

#endif

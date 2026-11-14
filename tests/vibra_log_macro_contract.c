/**
 * @file vibra_log_macro_contract.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#define LOG_TAG "TEST"
#include "vibra_log.h"

void exercise_vibra_log_macros(void)
{
  LOG_INIT();

#if defined(TEST_MASTER_OFF)
  LOG_E("M_ERROR=%u", VLOG_MASTER_OFF_ARGUMENT_MUST_NOT_EXIST);
#elif defined(TEST_WARN_ONLY)
  LOG_E("M_ERROR=%u", 1U);
  LOG_W("M_WARN");
  LOG_I("M_INFO=%u", VLOG_INFO_ARGUMENT_MUST_NOT_EXIST);
  LOG_D("M_DEBUG=%u", VLOG_DEBUG_ARGUMENT_MUST_NOT_EXIST);
  LOG_V("M_VERBOSE=%u", VLOG_VERBOSE_ARGUMENT_MUST_NOT_EXIST);
#else
  LOG_E("M_ERROR=%u", 1U);
  LOG_W("M_WARN");
  LOG_I("M_INFO");
  LOG_D("M_DEBUG=%x", 0x2AU);
  LOG_V("M_VERBOSE=%u", VLOG_VERBOSE_ARGUMENT_MUST_NOT_EXIST);
#endif
}

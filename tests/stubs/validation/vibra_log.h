/**
 * @file vibra_log.h
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef TEST_VALIDATION_LOG_H
#define TEST_VALIDATION_LOG_H
void test_log_info(const char *format, ...);
void test_log_error(const char *format, ...);
#define LOG_I(...) test_log_info(__VA_ARGS__)
#define LOG_E(...) test_log_error(__VA_ARGS__)
#define LOG_W(...) test_log_error(__VA_ARGS__)
#endif

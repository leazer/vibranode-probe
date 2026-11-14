/**
 * @file test_probe_time.c
 * @author 煲仔卤煮(ZealerLu)
 * @date 2026-09-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "probe_time.h"
#include <stdint.h>
uint32_t fake_counter, fake_pending, fake_primask, fake_wrap_on_read;

uint32_t tmr_counter_value_get(int timer)
{
    uint32_t value = fake_counter;
    (void)timer;
    if (fake_wrap_on_read) {
        fake_wrap_on_read = 0;
        fake_counter = 2;
        fake_pending = 1;
    }
    return value;
}

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
int test_main(void)
{
    uint64_t before, pending;
    unsigned i;
    probe_time_init();
    CHECK(probe_time_us() == 0);
    CHECK(fake_pending == 0);
    for (i = 0; i < 10; ++i) {
        fake_pending = 1;
        probe_time_overflow_isr();
    }
    fake_counter = 65535;
    before = probe_time_us();
    fake_counter = 2;
    fake_pending = 1;
    pending = probe_time_us();
    CHECK(pending == 11ULL * 65536 + 2);
    CHECK(pending > before);
    CHECK(probe_time_us() == pending);
    probe_time_overflow_isr();
    CHECK(probe_time_us() == pending);
    probe_time_overflow_isr(); /* spurious/shared IRQ must not advance time */
    CHECK(probe_time_us() == pending);
    fake_counter = 65535;
    fake_wrap_on_read = 1;
    CHECK(probe_time_us() == 12ULL * 65536 + 2);
    fake_primask = 1;
    CHECK(probe_time_us() == 12ULL * 65536 + 2);
    CHECK(fake_primask == 1);
    probe_time_init();
    CHECK(fake_primask == 1);
    CHECK(probe_time_us() == 0);
    return 0;
}

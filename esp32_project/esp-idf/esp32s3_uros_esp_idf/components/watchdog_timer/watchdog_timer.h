#ifndef WATCHDOG_TIMER_H
#define WATCHDOG_TIMER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t watchdog_time;
    int32_t timestep;
    bool timeout;
} WatchdogTimer;

void watchdog_timer_init(WatchdogTimer *wdt, int32_t interval);
bool watchdog_timer_check_timeout(WatchdogTimer *wdt);
void watchdog_timer_restart(WatchdogTimer *wdt);

#endif // WATCHDOG_TIMER_H

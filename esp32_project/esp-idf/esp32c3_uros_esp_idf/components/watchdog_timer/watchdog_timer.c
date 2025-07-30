#include "watchdog_timer.h"

void watchdog_timer_init(WatchdogTimer *wdt, int32_t interval)
{
    wdt->watchdog_time = 0;
    wdt->timestep = interval;
    wdt->timeout = false;
}

bool watchdog_timer_check_timeout(WatchdogTimer *wdt)
{
    wdt->watchdog_time++;
    if (wdt->watchdog_time > wdt->timestep)
    {
        wdt->timeout = true;
    }
    return wdt->timeout;
}

void watchdog_timer_restart(WatchdogTimer *wdt)
{
    wdt->watchdog_time = 0;
    wdt->timeout = false;
}

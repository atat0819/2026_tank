#ifndef WATCHDOG_HEARTBEAT_H
#define WATCHDOG_HEARTBEAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    WATCHDOG_TASK_CAN_SEND,
    WATCHDOG_TASK_UP_STAIR,
    WATCHDOG_TASK_REMOTE,
    WATCHDOG_TASK_GANTRY,
    WATCHDOG_TASK_COUNT
} WatchdogTask;

extern volatile uint32_t g_watchdog_heartbeats[WATCHDOG_TASK_COUNT];
void Watchdog_RecordHeartbeat(WatchdogTask task);

#ifdef __cplusplus
}
#endif

#endif

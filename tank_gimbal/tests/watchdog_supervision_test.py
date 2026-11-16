"""Exercise the real watchdog task with a simulated scheduler and IWDG."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class WatchdogSupervisionTest(unittest.TestCase):
    def test_startup_timeout_stall_and_tick_wrap(self):
        with tempfile.TemporaryDirectory() as temp:
            work = Path(temp)
            (work / "FreeRTOS.h").write_text(r'''
#pragma once
#include <stddef.h>
#include <stdint.h>
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
typedef int osPriority_t;
typedef void *osThreadId_t;
typedef struct { const char *name; unsigned stack_size; osPriority_t priority; } osThreadAttr_t;
enum { osPriorityNormal = 24, osPriorityAboveNormal = 32, pdPASS = 1 };
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
int xTaskCreate(TaskFunction_t, const char *, unsigned, void *, unsigned, TaskHandle_t *);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t);
osThreadId_t osThreadNew(TaskFunction_t, void *, const osThreadAttr_t *);
void osThreadExit(void);
void Error_Handler(void);
void MX_USB_DEVICE_Init(void);
void can_send_task(void *);
void gimbal_task(void *);
void remote_control_task(void *);
void gantry_task(void *);
''')
            (work / "iwdg.h").write_text(r'''
#pragma once
typedef struct { int unused; } IWDG_HandleTypeDef;
extern IWDG_HandleTypeDef hiwdg1;
enum { HAL_OK = 0, HAL_ERROR = 1 };
int HAL_IWDG_Refresh(IWDG_HandleTypeDef *);
''')
            for name in ("task.h", "main.h", "cmsis_os.h", "can_send_task.hpp", "gimbal_task.hpp",
                         "remote_control_task.hpp", "gantry_task.hpp", "usb_device.h"):
                (work / name).write_text("")
            source = (ROOT / "Core/Src/freertos.c").as_posix()
            (work / "test.c").write_text(r'''
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "FreeRTOS.h"
#include "iwdg.h"
#include "watchdog_heartbeat.h"
static TickType_t now, elapsed;
static int mode, stalled, feeds, last_feed, loops, max_loops, create_failure, create_calls;
static TaskFunction_t watchdog;
static jmp_buf done;
IWDG_HandleTypeDef hiwdg1;
TickType_t xTaskGetTickCount(void) { return now; }
int HAL_IWDG_Refresh(IWDG_HandleTypeDef *h) {
    assert(h == &hiwdg1);
    ++feeds; last_feed = (int)elapsed;
    return mode == 4 ? HAL_ERROR : HAL_OK;
}
void vTaskDelay(TickType_t ticks) {
    assert(ticks == 30);
    now += ticks; elapsed += ticks;
    for (int i = 0; i < WATCHDOG_TASK_COUNT; ++i) {
        if (mode == 0 && i == WATCHDOG_TASK_CAN_SEND && elapsed < 1600) continue;
        if (mode == 1 && i == stalled && elapsed >= 100 && elapsed < 400) continue;
        if (mode == 2 && i == WATCHDOG_TASK_CAN_SEND) continue;
        if (mode == 5 && ((i == WATCHDOG_TASK_CAN_SEND && elapsed < 1600) ||
                          (i == WATCHDOG_TASK_GIMBAL && elapsed >= 100))) continue;
        ++g_watchdog_heartbeats[i];
    }
    if (++loops >= max_loops) longjmp(done, 1);
}
int xTaskCreate(TaskFunction_t fn, const char *name, unsigned stack, void *arg,
                unsigned priority, TaskHandle_t *handle) {
    (void)stack; (void)arg; (void)priority; (void)handle;
    ++create_calls;
    if (strcmp(name, "Watchdog_Task") == 0) watchdog = fn;
    return create_calls == create_failure ? 0 : pdPASS;
}
osThreadId_t osThreadNew(TaskFunction_t fn, void *arg, const osThreadAttr_t *attr) {
    (void)fn; (void)arg; (void)attr; return create_failure == -1 ? NULL : (void *)1;
}
void Error_Handler(void) { longjmp(done, 2); }
void osThreadExit(void) {}
void MX_USB_DEVICE_Init(void) {}
void can_send_task(void *p) { (void)p; }
void gimbal_task(void *p) { (void)p; }
void remote_control_task(void *p) { (void)p; }
void gantry_task(void *p) { (void)p; }
''' + f'\n#include "{source}"\n' + r'''
static void run(int scenario, int target, TickType_t start, int iterations) {
    for (int i = 0; i < WATCHDOG_TASK_COUNT; ++i) g_watchdog_heartbeats[i] = 0;
    mode = scenario; stalled = target; now = start; elapsed = 0;
    feeds = 0; last_feed = -1; loops = 0; max_loops = iterations;
    if (setjmp(done) == 0) watchdog(NULL);
}
int main(void) {
    MX_FREERTOS_Init(); assert(watchdog != NULL);
    run(0, 0, 0, 100); assert(feeds == 100 && last_feed == 2970);
    for (int i = 0; i < WATCHDOG_TASK_COUNT; ++i) {
        run(1, i, 0, 30);
        assert(last_feed >= 240 && last_feed < 300);
        int stopped = feeds;
        assert(stopped < 20);
    }
    run(2, 0, 0, 180); assert(last_feed == 2970);
    run(5, 0, 0, 180); assert(last_feed == 2970);
    run(1, WATCHDOG_TASK_GIMBAL, UINT32_MAX - 60, 30);
    assert(last_feed >= 240 && last_feed < 300);
    run(4, 0, 0, 10); assert(feeds == 1);
    for (int i = -1; i <= 5; ++i) {
        if (i == 0) continue;
        create_failure = i; create_calls = 0;
        assert(setjmp(done) == 0 ? (MX_FREERTOS_Init(), 0) : 1);
    }
    return 0;
}
''')
            exe = work / "test.exe"
            subprocess.run(["gcc", "-std=c11", "-I", str(work), "-I", str(ROOT / "Core/Inc"),
                            str(work / "test.c"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()

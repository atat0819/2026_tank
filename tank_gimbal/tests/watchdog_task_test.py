import pathlib
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class WatchdogTaskTest(unittest.TestCase):
    def test_heartbeat_is_reported_after_each_task_cycle(self):
        cases = (
            ("can_send_task.cpp", "WATCHDOG_TASK_CAN_SEND", "vTaskDelay(1); // 相对延时"),
            ("gimbal_task.cpp", "WATCHDOG_TASK_GIMBAL", "vTaskDelay(5);"),
            ("gantry_task.cpp", "WATCHDOG_TASK_GANTRY", "osDelay(2);"),
            ("remote_control_task.cpp", "WATCHDOG_TASK_REMOTE", "vTaskDelay(2);"),
        )
        for filename, task_id, delay in cases:
            with self.subTest(task=filename):
                source = (ROOT / "RtosTask" / filename).read_text(encoding="utf-8-sig")
                loop = source[source.index("for(;;)" if filename != "remote_control_task.cpp" else "for (;;)"):]
                before_delay = loop[:loop.index(delay)]
                self.assertNotIn("continue;", before_delay)
                self.assertNotIn("return;", before_delay)
                self.assertIn(f"Watchdog_RecordHeartbeat({task_id});", before_delay)

    def test_task_is_created_above_control_priorities_and_delays(self):
        with tempfile.TemporaryDirectory() as temp:
            work = pathlib.Path(temp)
            (work / "FreeRTOS.h").write_text(r'''
#pragma once
#include <stddef.h>
typedef void *TaskHandle_t;
typedef unsigned TickType_t;
typedef void (*TaskFunction_t)(void *);
typedef int osPriority_t;
typedef void *osThreadId_t;
typedef struct { const char *name; unsigned stack_size; osPriority_t priority; } osThreadAttr_t;
enum { osPriorityNormal = 24, osPriorityAboveNormal = 32, pdPASS = 1 };
#define pdMS_TO_TICKS(ms) (ms)
void test_enter_critical(void);
void test_exit_critical(void);
#define taskENTER_CRITICAL() test_enter_critical()
#define taskEXIT_CRITICAL() test_exit_critical()
int xTaskCreate(TaskFunction_t, const char *, unsigned, void *, unsigned, TaskHandle_t *);
void vTaskDelay(unsigned);
TickType_t xTaskGetTickCount(void);
void Error_Handler(void);
osThreadId_t osThreadNew(TaskFunction_t, void *, const osThreadAttr_t *);
void osThreadExit(void);
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
enum { HAL_OK = 0 };
int HAL_IWDG_Refresh(IWDG_HandleTypeDef *);
''')
            for name in ("task.h", "main.h", "cmsis_os.h", "can_send_task.hpp",
                         "gimbal_task.hpp", "remote_control_task.hpp", "gantry_task.hpp", "usb_device.h"):
                (work / name).write_text("")
            source = (ROOT / "Core/Src/freertos.c").as_posix()
            (work / "test.c").write_text(r'''
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "FreeRTOS.h"
#include "iwdg.h"
#include "watchdog_heartbeat.h"
static TaskFunction_t watchdog;
static unsigned watchdog_priority, max_control_priority, delay_ticks;
static int critical_depth, critical_entries;
static jmp_buf stop;
IWDG_HandleTypeDef hiwdg1;
TickType_t xTaskGetTickCount(void) { return 0; }
int HAL_IWDG_Refresh(IWDG_HandleTypeDef *h) { (void)h; return HAL_OK; }
void Error_Handler(void) { assert(0); }
void test_enter_critical(void) { assert(critical_depth == 0); ++critical_depth; ++critical_entries; }
void test_exit_critical(void) { assert(critical_depth == 1); --critical_depth; }
int xTaskCreate(TaskFunction_t fn, const char *name, unsigned stack, void *arg,
                unsigned priority, TaskHandle_t *handle) {
    (void)stack; (void)arg; (void)handle;
    if (strcmp(name, "Watchdog_Task") == 0) {
        watchdog = fn;
        watchdog_priority = priority;
    }
    else if (priority > max_control_priority) max_control_priority = priority;
    return 1;
}
void vTaskDelay(unsigned ticks) { delay_ticks = ticks; longjmp(stop, 1); }
osThreadId_t osThreadNew(TaskFunction_t fn, void *arg, const osThreadAttr_t *attr) {
    (void)fn; (void)arg; (void)attr; return (void *)1;
}
void osThreadExit(void) {}
void MX_USB_DEVICE_Init(void) {}
void can_send_task(void *arg) { (void)arg; }
void gimbal_task(void *arg) { (void)arg; }
void remote_control_task(void *arg) { (void)arg; }
void gantry_task(void *arg) { (void)arg; }
''' + f'\n#include "{source}"\n' + r'''
int main(void) {
    for (int i = 0; i < WATCHDOG_TASK_COUNT; ++i) assert(g_watchdog_heartbeats[i] == 0);
    Watchdog_RecordHeartbeat(WATCHDOG_TASK_CAN_SEND);
    Watchdog_RecordHeartbeat(WATCHDOG_TASK_GIMBAL);
    Watchdog_RecordHeartbeat(WATCHDOG_TASK_GANTRY);
    Watchdog_RecordHeartbeat(WATCHDOG_TASK_REMOTE);
    Watchdog_RecordHeartbeat(WATCHDOG_TASK_CAN_SEND);
    assert(g_watchdog_heartbeats[WATCHDOG_TASK_CAN_SEND] == 2);
    assert(g_watchdog_heartbeats[WATCHDOG_TASK_GIMBAL] == 1);
    assert(g_watchdog_heartbeats[WATCHDOG_TASK_GANTRY] == 1);
    assert(g_watchdog_heartbeats[WATCHDOG_TASK_REMOTE] == 1);
    assert(critical_entries == 5 && critical_depth == 0);
    g_watchdog_heartbeats[WATCHDOG_TASK_CAN_SEND] = UINT32_MAX;
    Watchdog_RecordHeartbeat(WATCHDOG_TASK_CAN_SEND);
    assert(g_watchdog_heartbeats[WATCHDOG_TASK_CAN_SEND] == 0);
    MX_FREERTOS_Init();
    assert(watchdog != NULL);
    assert(watchdog_priority > max_control_priority);
    if (setjmp(stop) == 0) watchdog(NULL);
    assert(delay_ticks == 30);
    return 0;
}
''')
            exe = work / "test.exe"
            subprocess.run(["gcc", "-std=c11", "-I", str(work), "-I", str(ROOT / "Core/Inc"), str(work / "test.c"), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()

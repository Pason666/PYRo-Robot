#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

using namespace pyro;

extern "C"
{
    extern void pyro_init_thread(void *argument);
    extern void start_debug_task(void *arg);
    status_t pyro_init_ret;

    extern status_t sentry_gimbal_init(void *argument);
    extern status_t sentry_booster_init(void *argument);
    extern status_t sentry_chassis_init(void *argument);

    void start_mission_planner_task(void const *argument)
    {
        pyro_init_thread(NULL);

        #if BOARD == GIMBAL_BOARD
            pyro_init_ret = sentry_gimbal_init(nullptr);
            pyro_init_ret = sentry_booster_init(nullptr);
        #elif BOARD == CHASSIS_BOARD
            pyro_init_ret = sentry_chassis_init(nullptr);
        #endif

        #if DEBUG_MODE
            start_debug_task(NULL);
        #endif

        vTaskDelete(NULL);
    }
}
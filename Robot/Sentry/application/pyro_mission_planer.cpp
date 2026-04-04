#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "pyro_sentry_func_config.h"
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


    void start_mission_planer_task(void const *argument)
    {
        pyro_init_thread(NULL);

        #if BOARD_ID == GIMBAL_ID
            pyro_init_ret = sentry_gimbal_init(nullptr);
            pyro_init_ret = sentry_booster_init(nullptr);
        #elif BOARD_ID == CHASSIS_ID
            pyro_init_ret = sentry_chassis_init(nullptr);
        #endif

        #if DEBUG_MODE
            start_debug_task(NULL);
        #endif

        osThreadTerminate(NULL);
    }
}
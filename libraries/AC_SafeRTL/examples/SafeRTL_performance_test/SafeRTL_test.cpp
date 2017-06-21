/*
  generic Baro driver test
 */

#include <AC_SafeRTL/AC_SafeRTL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

void setup();
void loop();

void setup()
{
    hal.console->printf("SafeRTL performance test\n");

    AP_BoardConfig{}.init();

    hal.scheduler->delay(1000);
}

void loop()
{
    if (!hal.console->is_initialized()) {
        return;
    }

    uint32_t reference_time = AP_HAL::micros();

    hal.scheduler->delay(420); // TODO run something useful here.

    uint32_t run_time = AP_HAL::micros() - reference_time;
    hal.console->printf("Cleanup time: %u usec\n", run_time);

    hal.scheduler->delay(5000);
}

AP_HAL_MAIN();

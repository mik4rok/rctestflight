#include "SafeRTL_test.h"
#include <AC_SafeRTL/AC_SafeRTL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

Path* p;
uint32_t run_time;

void setup();
void loop();

void setup()
{
    hal.console->printf("SafeRTL performance test\n");
    AP_BoardConfig{}.init();
    hal.scheduler->delay(20000);

    p = new Path();
    p->path = rdp_test_before;
    uint32_t reference_time = AP_HAL::micros();

    p->_rdp(0,98,RDP_EPSILON);
    // TODO do a memcmp to see if the output is actually correct

    run_time = AP_HAL::micros() - reference_time;
}

void loop()
{
    if (!hal.console->is_initialized()) {
        return;
    }
    hal.console->printf("Cleanup time: %u usec\n", run_time);

    hal.scheduler->delay(5000);
}

AP_HAL_MAIN();

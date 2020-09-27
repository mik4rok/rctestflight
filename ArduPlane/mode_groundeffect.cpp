#include "mode.h"
#include "Plane.h"

bool ModeGroundEffect::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    // TODO maybe fail if the altitude reading is too big

    return true;
}

void ModeGroundEffect::update()
{
    // Desired roll is flat (0 hundredths of a degree). Note that the pilot can have some authority if stick_mixing is enabled
    plane.nav_roll_cd = 0;
    // Desired pitch is 10 degrees nose-up (1000 hundredths of a degree).
    plane.nav_pitch_cd = 1000; // FIXME should this be negative?
    // Pilot has standard manual control authority over rudder
    plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

    /*
    * TODO implement closed-loop altitude-throttle control here
    * Probably create a new controller using AC_PID
    * And get range from plane.rangefinder_state.last_distance. I think this is in cm and updates at 10Hz
    * Not sure about the best way to set throttle. What happens when I set auto_throttle_mode=True?
    */
}


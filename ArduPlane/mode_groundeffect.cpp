#include "mode.h"
#include "Plane.h"

bool ModeGroundEffect::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;// This runs the TECS speed/height controller. Not needed.
    plane.auto_navigation_mode = false;

    // // Verify that a mm-precision-capable, downward-facing rangefinder is configured
    if(!plane.rangefinder.has_mm_prec_orient(ROTATION_PITCH_270)){
        return false;
    } 

    // The nominal throttle (throttle at zero error) should be the midpoint between the high and low throttle parameters
    _thr_ff = (plane.g.gndEffect_thr_max + plane.g.gndEffect_thr_min)/2.f;

    // The desired altitude should be the midpoint between the high and low altitude parameters
    _alt_desired_mm = (plane.g.gndEffect_alt_max + plane.g.gndEffect_alt_min)/2;

    float new_thr_p = ((float) (plane.g.gndEffect_thr_max - plane.g.gndEffect_thr_min)) / ((float) (plane.g.gndEffect_alt_max - plane.g.gndEffect_alt_min));
    plane.g2.gndefct_thr.kP(new_thr_p);

    plane.g2.gndefct_thr.reset();
    plane.g2.gndefct_ele.reset();
    plane.g2.gndefct_flaps.reset();

    return true;
}

void ModeGroundEffect::update()
{
    // This method runs at 400Hz. Rangefinder probably senses at 30Hz
    int16_t errorMm = _alt_desired_mm - plane.rangefinder.distance_mm_orient(ROTATION_PITCH_270);

    // Wings level. Note that the pilot can also have some authority if stick_mixing is enabled
    plane.nav_roll_cd = 0;
    // Desired nose-up pitch
    plane.nav_pitch_cd = (int16_t) plane.g2.gndefct_ele.get_pid(errorMm);
    // Pilot has standard manual control of rudder
    plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

    // flaps are actually set in servos.cpp using this number
    desired_flap_percentage = (uint8_t) constrain_int16(plane.g2.gndefct_flaps.get_pid(errorMm), -100, 100);

    // If the rc throttle input is zero, don't run throttle controller
    // This allows the user to stop flight by reflexively cutting the throttle
    if(plane.get_throttle_input(false) == 0){
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        return;
    }

    int16_t throttle_command = plane.g2.gndefct_thr.get_pid(errorMm) + _thr_ff;

    int16_t commanded_throttle = constrain_int16(throttle_command, plane.g.gndEffect_thr_min, plane.g.gndEffect_thr_max);
    commanded_throttle = constrain_int16(commanded_throttle, 0, 100);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}

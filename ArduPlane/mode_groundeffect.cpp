#include "mode.h"
#include "Plane.h"

/*
*   The desired altitude to hover at, in centimeters.
*   This number should probably be around 1/4 of the wingspan or less.
*/
// static constexpr float GROUND_EFFECT_TARGET_ALT_CM{7.0};

/*
*   The steady-state throttle required at the given altitude to remain in that altitude.
*   This number must be between 0 and 100.
*/
// static constexpr int16_t GROUND_EFFECT_STEADY_THROTTLE{70};

/*
*   The nose-up pitch to hold the vehicle at, in centidegrees
*   Consider that a steep pitch might cause the rangefinder to give bad readings.
*   For example, the FoV of the VL53L0X is 25 degrees, so pitching anywhere near 25/2 will cause trouble.
*/
// static constexpr int32_t GROUND_EFFECT_PITCH_CENTIDEGREES{0};

/*
*   The P gain for the Alt2Throttle P controller.
*   For every centimeter below the target altitude, throttle increases by this many percent
*/
// static constexpr float GROUND_EFFECT_CONTROLLER_KP{7.0}; // 80 at the ground

// TODO make these real parameters

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

    // TODO not sure if this is how you control the flaperons
    int8_t flap_percentage = (uint8_t) constrain_int16(plane.g2.gndefct_flaps.get_pid(errorMm), -100, 100);
    plane.flaperon_update(flap_percentage);

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

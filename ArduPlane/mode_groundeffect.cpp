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
static constexpr int32_t GROUND_EFFECT_PITCH_CENTIDEGREES{0};

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

    // // Verify that the rangefinder is capable of measuring short distances. This fails with most rangefinders.
    // if(plane.rangefinder.min_distance_cm_orient(ROTATION_PITCH_270) >= 10){
    //     return false;
    // }

    // // Verify rangefinder health: this fails if we are low or high or no sensor data
    // if(plane.rangefinder.status_orient(ROTATION_PITCH_270) != RangeFinder::Status::Good){
    //     return false;
    // }

    // // Verify rangefinder health: Last reading should not be more than 0.5 seconds old
    // if((AP_HAL::millis() - plane.rangefinder.last_reading_ms(ROTATION_PITCH_270)) > 500){
    //     return false;
    // }

    // // Verify that we are somewhat close to the desired ground effect altitude
    // if(plane.rangefinder.distance_cm_orient(ROTATION_PITCH_270) > 3 * GROUND_EFFECT_TARGET_ALT_CM){
    //     return false;
    // }
    
    return true;
}

void ModeGroundEffect::update()
{
    // Desired roll is flat (0 hundredths of a degree). Note that the pilot can have some authority if stick_mixing is enabled
    plane.nav_roll_cd = 0;
    // Desired nose-up pitch
    plane.nav_pitch_cd = GROUND_EFFECT_PITCH_CENTIDEGREES;
    // Pilot has standard manual control of rudder
    plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

    // If the rc throttle input is zero, don't run throttle controller
    // This allows the user to stop flight by reflexively cutting the throttle
    if(plane.get_throttle_input(false) == 0){
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        return;
    }
    
    // This method runs at 400Hz. Rangefinder probably senses at 30Hz
    uint16_t altMm = plane.rangefinder.distance_mm_orient(ROTATION_PITCH_270);

    // Slope: How much should throttle % decrease for every mm increase in alt
    float   m = -((float) (plane.g.gndEffect_thr_max - plane.g.gndEffect_thr_min)) / ((float) (plane.g.gndEffect_alt_max - plane.g.gndEffect_alt_min));// -(80-10)/(220-100) = 90/120=-0.75
    // Alt: The altitude in mm above the minimum altitude
    float   x = altMm - plane.g.gndEffect_alt_min; // alt between 100 and 220
    // Intercept: How many % should the throttle be shifted up
    int16_t b = plane.g.gndEffect_thr_max;

    int16_t y = m*x+b;

    int16_t commanded_throttle = constrain_int16(y, plane.g.gndEffect_thr_min, plane.g.gndEffect_thr_max);
    commanded_throttle = constrain_int16(commanded_throttle, 0, 100);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}


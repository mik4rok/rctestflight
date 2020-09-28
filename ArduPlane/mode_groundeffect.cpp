#include "mode.h"
#include "Plane.h"

/*
*   The desired altitude to hover at, in centimeters.
*   This number should probably be around 1/4 of the wingspan or less.
*/
static constexpr float GROUND_EFFECT_TARGET_ALT_CM{5.0};

/*
*   The steady-state throttle required at the given altitude to remain in that altitude.
*   This number must be between 0 and 100.
*/
static constexpr int16_t GROUND_EFFECT_STEADY_THROTTLE{50};

/*
*   The nose-up pitch to hold the vehicle at, in centidegrees
*   Consider that a steep pitch might cause the rangefinder to give bad readings.
*   For example, the FoV of the VL53L0X is 25 degrees, so pitching anywhere near 25/2 will cause trouble.
*/
static constexpr int32_t GROUND_EFFECT_PITCH_CENTIDEGREES{1000};

/*
*   The P gain for the Alt2Throttle P controller.
*   For every centimeter below the target altitude, throttle increases by this many percent
*/
static constexpr float GROUND_EFFECT_CONTROLLER_KP{10.0};

bool ModeGroundEffect::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;// This runs the TECS speed/height controller. Not needed.
    plane.auto_navigation_mode = false;

    // Verify that a downward-facing rangefinder is configured
    if(!plane.rangefinder.has_orientation(ROTATION_PITCH_270)){
        return false;
    }

    // Verify that the rangefinder is capable of measuring short distances. This fails with most rangefinders.
    if(plane.rangefinder.min_distance_cm_orient(ROTATION_PITCH_270) >= 10){
        return false;
    }

    // Verify rangefinder health: this fails if we are low or high or no sensor data
    if(plane.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good){
        return false;
    }

    // Verify rangefinder health: Last reading should not be more than 0.5 seconds old
    if((plane.rangefinder.last_reading_ms(ROTATION_PITCH_270) - AP_HAL::millis()) > 500){
        return false;
    }

    // Verify that we are somewhat close to the desired ground effect altitude
    if(plane.rangefinder.distance_cm_orient(ROTATION_PITCH_270) > 3 * GROUND_EFFECT_TARGET_ALT_CM){
        return false;
    }

    // Set the gain for the P controller
    pAlt2Throttle(GROUND_EFFECT_CONTROLLER_KP);

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

    /*
    *   TODO
    *   Consider filtering rangefinder output. See ../libraries/Filter
    *   Rangefinder is read at 50Hz. This method runs at 400Hz
    *   Also handle repeated bad readings somehow. Maybe disarm
    */
  
    float error = GROUND_EFFECT_TARGET_ALT_CM - plane.rangefinder.distance_cm_orient(ROTATION_PITCH_270);

    int16_t commanded_throttle = GROUND_EFFECT_STEADY_THROTTLE + ((int16_t) pAlt2Throttle.get_p(error));

    // commanded_throttle should be in the range 0...100
    commanded_throttle = constrain_int16(commanded_throttle, 0, 100);

    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}


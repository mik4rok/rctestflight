/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//

#include <AP_HAL/AP_HAL.h>
#include "AP_GroundEffectController.h"
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_GroundEffectController::var_info[] = {
    // index 5, 6 reserved for old IMAX, FF

    // @Param: _RATE_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(pid, "_RATE_", 9, AP_GroundEffectController, AC_PID),
    
    AP_GROUPEND
};

// constructor
AP_GroundEffectController::AP_GroundEffectController()
{
    return;
}

bool AP_GroundEffectController::user_request_enable(bool enable)
{
    // aux channel should call this frequently
    // if rising edge {
    // if(!plane.rangefinder.has_mm_prec_orient(ROTATION_PITCH_270)){
    //     return false;
    // }
    reset();
    return true;
}

void AP_GroundEffectController::reset()
{
    // TODO reset complementary filter
    // TODO reset pid I term for pitch and throttle pid controllers

    // The nominal throttle (throttle at zero error) should be the midpoint between the high and low throttle parameters
    // _thr_ff = (plane.g.gndEffect_thr_max + plane.g.gndEffect_thr_min)/2.f;

    // // The desired altitude should be the midpoint between the high and low altitude parameters
    // _alt_desired_mm = (plane.g.gndEffect_alt_max + plane.g.gndEffect_alt_min)/2;

    // float new_thr_p = ((float) (plane.g.gndEffect_thr_max - plane.g.gndEffect_thr_min)) / ((float) (plane.g.gndEffect_alt_max - plane.g.gndEffect_alt_min));
    // plane.g2.gndefct_thr.kP(new_thr_p);

    // plane.g2.gndefct_thr.reset();
    // plane.g2.gndefct_ele.reset();
    // plane.g2.gndefct_flaps.reset();
    return;
}

void AP_GroundEffectController::update()
{
    // TODO read rangefinder. put it into complementary filter

/*
    // get our new reading, if possible
    if(plane.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good){
        _last_good_reading_mm = plane.rangefinder.distance_mm_orient(ROTATION_PITCH_270);
        _last_good_reading_time_ms = AP_HAL::millis();
    }

    // If we haven't gotten a new reading in a while, assume that we are high and inject a high reading
    if(AP_HAL::millis() - _last_good_reading_time_ms > 1000) {
        _last_good_reading_mm = plane.g.gndEffect_alt_max;
    }

    // This method runs at 400Hz. Rangefinder probably senses at 30Hz
    int16_t errorMm = _alt_desired_mm - _last_good_reading_mm;

    // Wings level. Note that the pilot can also have some authority if stick_mixing is enabled
    plane.nav_roll_cd = 0;
    // Desired nose-up pitch
    plane.nav_pitch_cd = (int16_t) plane.g2.gndefct_ele.get_pid(errorMm);
    // Pilot has standard manual control of rudder
    plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

    // flaps are actually set in servos.cpp using this number
    desired_flap_percentage = (int8_t) constrain_int16(plane.g2.gndefct_flaps.get_pid(errorMm), -100, 100);

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
*/
    return;
}

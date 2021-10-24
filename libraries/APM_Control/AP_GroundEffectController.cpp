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

//	Code by Sebastian Quilter

#include <AP_HAL/AP_HAL.h>
#include "AP_GroundEffectController.h"
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_GroundEffectController::var_info[] = {
    // @Param: P
    // @DisplayName: P gain
    // @Description: P gain
    // @User: Standard

    // @Param: I
    // @DisplayName: I gain
    // @Description: I gain
    // @User: Standard

    // @Param: D
    // @DisplayName: D gain
    // @Description: D gain
    // @User: Standard

    // @Param: IMAX
    // @DisplayName: IMax
    // @Description: Maximum integrator value
    // @User: Standard
    AP_SUBGROUPINFO(_throttle_pid, "_THR_", 1, AP_GroundEffectController, PID),

        // @Param: P
    // @DisplayName: P gain
    // @Description: P gain
    // @User: Standard

    // @Param: I
    // @DisplayName: I gain
    // @Description: I gain
    // @User: Standard

    // @Param: D
    // @DisplayName: D gain
    // @Description: D gain
    // @User: Standard

    // @Param: IMAX
    // @DisplayName: IMax
    // @Description: Maximum integrator value
    // @User: Standard
    AP_SUBGROUPINFO(_pitch_pid, "_PITCH_", 2, AP_GroundEffectController, PID),

    AP_GROUPEND
};

bool AP_GroundEffectController::user_request_enable(bool enable)
{
    if(enable){
        if(!_rangefinder.has_orientation(ROTATION_PITCH_270)){
            _enabled = false;
            return false;
        }
        reset();
    }

    _enabled = enable;
    return true;
}

void AP_GroundEffectController::reset()
{
    _altFilter.set_cutoff_frequency(0.05); // TODO should be parameter!
    _altFilter.reset();
    // TODO reset complementary filter
    // TODO reset pid I term for pitch and throttle pid controllers

    // The nominal throttle (throttle at zero error) should be the midpoint between the high and low throttle parameters
    // _thr_ff = (plane.g.gndEffect_thr_max + plane.g.gndEffect_thr_min)/2.f;

    // // The desired altitude should be the midpoint between the high and low altitude parameters
    // _alt_desired_mm = (plane.g.gndEffect_alt_max + plane.g.gndEffect_alt_min)/2;

    // float new_thr_p = ((float) (plane.g.gndEffect_thr_max - plane.g.gndEffect_thr_min)) / ((float) (plane.g.gndEffect_alt_max - plane.g.gndEffect_alt_min));
    // plane.g2.gndefct_thr.kP(new_thr_p);
    _pitch_pid.reset_I();
    _throttle_pid.reset_I();
    return;
}

void AP_GroundEffectController::update()
{
    float ahrs_alt;
    bool ahrs_ok = _ahrs.get_relative_position_D_origin(ahrs_alt);
    (void) ahrs_ok;
    // TODO what if ahrs_ok isn't ok? what if plane.rangefinder.status_orient(ROTATION_PITCH_270) != RangeFinder::Status::Good?
    _altFilter.apply(_rangefinder.distance_orient(ROTATION_PITCH_270), ahrs_alt, AP_HAL::micros());

    // TODO give error to these controllers. actual minus desired.

    float error = _altFilter.get(); // subtract expected. maybe minus?

    _pitch = _pitch_pid.get_pid(error);
    _throttle = _throttle_pid.get_pid(error); // plus ff

    // TODO make these parameters exist
    //_throttle = constrain_int16(_throttle, plane.g.gndEffect_thr_min, plane.g.gndEffect_thr_max);

    return;
}

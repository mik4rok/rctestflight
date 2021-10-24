#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <PID/PID.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <Filter/ComplementaryFilter.h>

class AP_GroundEffectController {
public:
    AP_GroundEffectController(AP_AHRS& ahrs, RangeFinder& rangefinder)
        : _ahrs{ahrs}
        , _rangefinder{rangefinder}
        , _enabled{false} {};

    /* Do not allow copies */
    AP_GroundEffectController(const AP_GroundEffectController &other) = delete;
    AP_GroundEffectController &operator=(const AP_GroundEffectController&) = delete;

    bool user_request_enable(bool enable);

    bool enabled_by_user() { return _enabled; }

    void update();

	void reset();

    const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

    int32_t get_pitch() { return _pitch; }

    int16_t get_throttle() { return _throttle; }

private:
    // TODO set reasonable numbers here
    PID _pitch_pid{0.04, 0.15, 0, 1000};
    PID _throttle_pid{0.04, 0.15, 0, 1000};

    AP_Logger::PID_Info _pid_info;

    AP_AHRS& _ahrs;
    RangeFinder& _rangefinder;
    ComplementaryFilter _altFilter; // should be reference?

    bool _enabled;
    int32_t _pitch;
    int16_t _throttle;
};

/*
Thoughts:
- auto mode should refuse to accept the mission if the AP_GroundEffectController doesn't exist
- user aux switch can enable/disable ground effect submode
- maybe move all of this to AP_GroundEffect

*/
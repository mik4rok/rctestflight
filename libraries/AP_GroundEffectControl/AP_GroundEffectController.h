#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <PID/PID.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <Filter/ComplementaryFilter.h>

class GroundEffectController {
public:
    GroundEffectController(AP_AHRS& ahrs, RangeFinder& rangefinder)
        : _ahrs{ahrs}
        , _rangefinder{rangefinder}
        , _enabled{false} {};

    /* Do not allow copies */
    GroundEffectController(const GroundEffectController &other) = delete;
    GroundEffectController &operator=(const GroundEffectController&) = delete;

    bool user_request_enable(bool enable);

    bool enabled_by_user() { return _enabled; }

    void update();

	void reset();

    const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

    int32_t get_pitch() { return _pitch; }

    int16_t get_throttle() { return _throttle; }

private:
    // TODO set reasonable numbers here. Also set all other default params and ranges and descriptions
    PID _pitch_pid{0.04, 0.15, 0, 1000};
    PID _throttle_pid{0.04, 0.15, 0, 1000};

	AP_Float _THR_REF;
    AP_Float _THR_MIN;
    AP_Float _THR_MAX;
    AP_Float _ALT_REF;
    AP_Float _CUTOFF_FREQ;

    AP_Logger::PID_Info _pid_info;

    uint32_t _last_time_called;

    AP_AHRS& _ahrs;
    RangeFinder& _rangefinder;
    ComplementaryFilter _altFilter;

    float _last_good_ahrs_reading;
    float _last_good_rangefinder_reading;

    bool _enabled;
    int32_t _pitch;
    int16_t _throttle;
};

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <Filter/ComplementaryFilter.h>

class AP_GroundEffectController {
public:
    AP_GroundEffectController();

    /* Do not allow copies */
    AP_GroundEffectController(const AP_GroundEffectController &other) = delete;
    AP_GroundEffectController &operator=(const AP_GroundEffectController&) = delete;

    bool user_request_enable(bool enable);

    bool enabled_by_user() { return false; }

    void update();

	void reset();

    const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

    int32_t get_pitch() { return _pitch; }

    int16_t get_throttle() { return _throttle; }

private:
    AC_PID pid{0.04, 0.15, 0, 0.345, 0.666, 3, 0, 12, 0.02, 150, 1};

    AP_Logger::PID_Info _pid_info;

    ComplementaryFilter _filteredRangefinder; // should be reference?

    int32_t _pitch;
    int16_t _throttle;
};

/*
Thoughts:
- auto mode should refuse to accept the mission if the AP_GroundEffectController doesn't exist
- user aux switch can enable/disable ground effect submode
- maybe move all of this to AP_GroundEffect

*/
#include "Copter.h"

/*
 * Init and run calls for Safe_RTL flight mode
 *
 * This code uses the SafeRTL path that is already in memory, and feeds it into WPNav, one point at a time.
 * Once the copter is close to home, it will run a pretty standard land controller.
 */

bool Copter::safe_rtl_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        safe_rtl_state = SafeRTL_WaitForCleanup;
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();
        // stay in place for now
        wp_nav->init_loiter_target(); // TODO this isn't stopping in place. am i supposed to do this? wp_nav->init_loiter_target(wp_nav->get_wp_destination());
        // look in copter for stopping point or get_wp_destination. make sure to set destination to current, so that next part is not held up by reached_wp_destination()

        // initialise yaw to obey user parameter
        set_auto_yaw_mode(get_default_auto_yaw_mode(true));

        // tell library to stop accepting new breadcrumbs
        safe_rtl_path.accepting_new_points = false;
        return true;
    }else{
        return false;
    }
}

void Copter::safe_rtl_run()
{
    switch(safe_rtl_state){
        case SafeRTL_WaitForCleanup:
            safe_rtl_wait_cleanup();
            break;
        case SafeRTL_PathFollow:
            safe_rtl_path_follow();
            break;
        case SafeRTL_PreLandPosition:
            safe_rtl_pre_land_position();
            break;
        case SafeRTL_Land:
            safe_rtl_land();
            break;
    }
}

void Copter::safe_rtl_wait_cleanup()
{
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());

    if (safe_rtl_path.cleanup_ready()){
        safe_rtl_path.thorough_cleanup();
        safe_rtl_state = SafeRTL_PathFollow;
    }
}

void Copter::safe_rtl_path_follow()
{
    // if we are close to current target point, switch the next point to be our target.
    if(wp_nav->reached_wp_destination()){
        Vector3f next_point = safe_rtl_path.pop_point();
        if (next_point != Vector3f{0.0f, 0.0f, 0.0f}){
            next_point[0] *= 100.0f;
            next_point[1] *= 100.0f;
            next_point[2] *= -100.0f; // invert because this next method wants cm NEU
            // gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "SafeRTL going to: %f %f %f", next_point[0], next_point[1], next_point[2]);
            wp_nav->set_wp_destination(next_point, false);
        } else {
            // go to the point that is 1m above home, instead of directly home.
            wp_nav->set_wp_destination(Vector3f{0.0f, 0.0f, 200.0f}, false); // {0,0,-2} in NED
            safe_rtl_state = SafeRTL_PreLandPosition;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    // TODO figure out heading. Should it follow that parameter? Probably best to copy behavior rtl
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
}

void Copter::safe_rtl_pre_land_position()
{
    // if we are close to {0,0,-2}, we are ready to land.
    if(wp_nav->reached_wp_destination()){
        safe_rtl_state = SafeRTL_Land;
    }
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
}

void Copter::safe_rtl_land()
{
    // TODO do this better. Try to reuse rtl_land_init() and rtl_land_run()
    set_mode(LAND, MODE_REASON_UNKNOWN);
}

/**
*   This method should be run a couple times per second.
*/
void Copter::safe_rtl_drop_breadcrumb()
{
    Vector3f current_pos {};
    if(ahrs.get_relative_position_NED_origin(current_pos)){ // meters from origin, NED
        // it's important to do the cleanup first, because appending a point will reset the cleanup methods,
        // so there will not be anything to clean up immediately after adding a point.
        // The cleanup usually returns immediately. If it decides to actually perform the cleanup, it takes about 100us.
        safe_rtl_path.routine_cleanup();
        safe_rtl_path.append_if_far_enough(current_pos);
    }
}

/**
*   This method might take longer than 1ms. It should be run as often as possible,
*   ideally not in the main loop.
*/
void Copter::safe_rtl_background_cleanup()
{
    safe_rtl_path.detect_loops(300);
    safe_rtl_path.rdp(200);
}

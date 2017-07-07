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
        safe_rtl_state = SafeRTL_PathFollow;
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();
        // stay in place for now
        Vector3f current_pos;
        ahrs.get_relative_position_NED_origin(current_pos);
        wp_nav->set_wp_destination(current_pos, false);
        // initialise yaw
        set_auto_yaw_mode(AUTO_YAW_HOLD);
        // tell library to stop accepting new breadcrumbs
        safe_rtl_path.accepting_new_points = false;
        return true;
    }else{
        return false;
    }
}

void Copter::safe_rtl_run()
{
    // if cleanup algorithms aren't ready, stay in place
    if (!safe_rtl_path.cleanup_ready()){
        wp_nav->update_wpnav();
        pos_control->update_z_controller();
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
        return;
    }

    // if we are within 1 meter of current target point, switch the next point to be our target.
    if (safe_rtl_state == SafeRTL_PathFollow && wp_nav->get_wp_distance_to_destination() <= 100.0f){ // TODO parameterize 100
        Vector3f next_point = safe_rtl_path.pop_point();
        if (next_point != Vector3f{0.0f, 0.0f, 0.0f}){
            next_point[0] *= 100.0f;
            next_point[1] *= 100.0f;
            next_point[2] *= -100.0f; // invert because this next method wants cm NEU
            wp_nav->set_wp_destination(next_point, false);
        } else {
             // go to the point that is 1m above home, instead of directly home.
            wp_nav->set_wp_destination({0.0f, 0.0f, 100.0f}, false); // {0,0,-1} in NED
            safe_rtl_state = SafeRTL_PreLandPosition;
        }
    } else if (safe_rtl_state == SafeRTL_PreLandPosition && wp_nav->get_wp_distance_to_destination() <= 1.0f) {
        rtl_land_start(); // FIXME this method will mess with rtl_state and rtl_state_complete
        safe_rtl_state = SafeRTL_Land;
    } else if (safe_rtl_state == SafeRTL_Land) {
        rtl_land_run(); // FIXME same here
        return;
    }

    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
}

void Copter::safe_rtl_cleanup()
{
    //gcs_send_text(MAV_SEVERITY_NOTICE, "debug");
    safe_rtl_path.detect_loops(300);
    safe_rtl_path.rdp(200);
}

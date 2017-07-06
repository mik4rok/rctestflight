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
        // TODO tell controller to stay in place. Important that get_wp_distance_to_destination() will report almost-zero
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
    if (safe_rtl_path.cleanup_ready()){
        wp_nav->update_wpnav();
        return;
    }

    // if we are within 1 meter of current target point, switch the next point to be our target.
    if (safe_rtl_state == SafeRTL_PathFollow && p_nav->get_wp_distance_to_destination() <= 1.0f){ // TODO parameterize
        Vector3f next_point = safe_rtl_path.pop();
        if (next_point != {0.0f, 0.0f, 0.0f}){
            wp_nav->set_wp_destination(next_point, false);
        } else {
             // go to the point that is 1m above home, instead of directly home.
            wp_nav->set_wp_destination({0.0f, 0.0f, -1.0f}, false);
            safe_rtl_state = SafeRTL_PreLandPosition;
        }
    } else if (safe_rtl_state == SafeRTL_PreLandPosition && p_nav->get_wp_distance_to_destination() <= 1.0f) {
        rtl_land_start(); // FIXME this method will mess with rtl_state and rtl_state_complete
        safe_rtl_state = SafeRTL_Land;
    } else if (safe_rtl_state == SafeRTL_Land) {
        rtl_land_run();
        return;
    }

    update_wpnav();
}

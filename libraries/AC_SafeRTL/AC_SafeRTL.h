#pragma once

#include <AP_Buffer/AP_Buffer.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <bitset> // TODO if this uses a non-negligible amount of memory, re-implement without this
#include <vector>

#define SMALL_FLOAT  0.0000001f

#define POSITION_DELTA 2.0f // how many meters to move before appending a new position to return_path
#define PRUNING_DELTA (POSITION_DELTA * 1.5) //h ow many meteres apart must two points be, such that we can assume that there is no obstacle between those points
#define RDP_EPSILON (POSITION_DELTA * 0.5)
#define MAX_PATH_LEN 100 // the amount of memory used by safe RTL will be slightly higher than 3*8*MAX_PATH_LEN bytes. Increasing this number will improve path pruning, but will use more memory, and running a path cleanup will take longer. No longer than 255.
#define RDP_STACK_LEN 64 // the amount of memory to be allocated for the RDP algorithm to write its to do list.
// XXX A number too small for RDP_STACK_LEN can cause a buffer overflow! The number to put here is int((s/2-1)+min(s/2, MAX_PATH_LEN-s)), where s = pow(2, floor(log(MAX_PATH_LEN)/log(2)))
// To avoid this annoying math, a good-enough overestimate is ciel(MAX_PATH_LEN*2./3.)

#define HYPOT(a,b) (a-b).length()

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class Path {
    // points are stored in meters from EKF origin in NED
    Vector3f path[MAX_PATH_LEN];
    int _last_index;
public:
    Path();
    void append_if_far_enough(Vector3f);
    void routine_cleanup();
    Vector3f* thorough_cleanup();
    bool accepting_new_points; // false means that any call to append_if_far_enough() will fail. This should be unset when entering SafeRTL mode, and set when exiting.
private:
    // the two cleanup steps. These should be run regularly, maybe even by a different thread
    void _rdp(uint32_t);
    void _detect_loops(uint32_t);
    // misc cleanup helper methods:
    void _zero_points_by_simplification_bitmask();
    void _zero_points_by_loops();
    void _remove_empty_points();
    // _segment_segment_dist returns two things, the closest distance reached between 2 line segments, and the point exactly between them.
    typedef struct dist_point {
        float distance;
        Vector3f point;
    } dist_point;
    static dist_point _segment_segment_dist(Vector3f, Vector3f, Vector3f, Vector3f);
    static float _point_line_dist(Vector3f, Vector3f, Vector3f);

    // Simplification state
    bool _simplification_complete;
    // structure and buffer to hold the "to-do list" for the RDP algorithm.
    struct start_finish {
        uint8_t start;
        uint8_t finish;
    };
    AP_Buffer<start_finish,RDP_STACK_LEN> _simplification_stack;
    // the result of the simplification algorithm
    std::bitset<MAX_PATH_LEN>() _simplification_bitmask;

    // Pruning state
    bool _pruning_complete;
    int _pruning_current_i;
    int _pruning_min_j;
    typedef struct loop {
        uint8_t start_index;
        uint8_t end_index;
        Vector3f halfway_point;
    } loop;
    // the result of the pruning algorithm
    std::vector<loop> _prunable_loops; // TODO this might allocate memory while flying. Maybe consider a way to do this with an array or AP_Buffer
};

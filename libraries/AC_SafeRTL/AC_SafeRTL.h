#pragma once

#include <AP_Math/AP_Math.h>
#include <bitset> // TODO if this uses a non-negligible amount of memory, reimplement without this

#define SMALL_FLOAT  0.0000001

#define POSITION_DELTA 2.0f // how many meters to move before appending a new position to return_path
#define PRUNING_DELTA (POSITION_DELTA * 1.5) //h ow many meteres apart must two points be, such that we can assume that there is no obstacle between those points
#define RDP_EPSILON (POSITION_DELTA * 0.5)
#define MAX_PATH_LEN 100 // the amount of memory used by safe RTL will be slightly higher than 3*8*MAX_PATH_LEN bytes. Increasing this number will improve path pruning, but will use more memory, and running a path cleanup will take longer. No longer than 255.
#define RDP_STACK_LEN 64 // the amount of memory to be allocated for the RDP algorithm to write its to do list.
// XXX A number too small for RDP_STACK_LEN can cause a buffer overflow! The number to put here is int((s/2-1)+min(s/2, MAX_PATH_LEN-s)), where s = pow(2, floor(log(MAX_PATH_LEN)/log(2)))
// To avoid this annoying math, a good-enough estimate is ciel(MAX_PATH_LEN*2./3.)

// dot product is already defined using the * operator with Vector3f objects
// cross product is %. length() also exists

#define HYPOT(a,b) (a-b).length()

typedef struct start_finish {
    uint8_t start;
    uint8_t finish;
} start_finish;

class RDP_Stack {
    start_finish * stack;
    start_finish * top;
public:
    RDP_Stack();
    void push(start_finish);
    start_finish pop();
    bool empty();
};

typedef struct dist_point {
    float distance;
    Vector3f point;
} dist_point;

class Path {
    // points are stored in meters from EKF origin in NED
    int last_index, worst_length;
    RDP_Stack * stack;
public:
    Vector3f * path; // by leaving this public, we can inject data into here easily which helps to benchmark algorithm performance. FIXME move it back.
    Path();
    void append_if_far_enough(Vector3f);
    void routine_cleanup();
    void thorough_cleanup();
    int _rdp(uint8_t, float); // FIXME move this back to private. It's here now for easy benchmarking
private:
    bool _cleanup();
    static dist_point _segment_segment_dist(Vector3f, Vector3f, Vector3f, Vector3f);
    static float _point_line_dist(Vector3f, Vector3f, Vector3f);
};

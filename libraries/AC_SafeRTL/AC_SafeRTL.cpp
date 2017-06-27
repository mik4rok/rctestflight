#include "AC_SafeRTL.h"

/*
*    This library is used for copter's Safe Return-to-Launch feature. It stores
*    "breadcrumbs" in memory, up to a certain number  of breadcrumbs have been stored,
*    and then cleans up those breadcrumbs when space is filling up. When Safe-RTL is
*    triggered, a more thorough cleanup occurs, and then the resulting path can
*    be fed into a position controller to fly it.
*
*    The cleanup consists of two parts, pruning and simplification. Pruning works
*    by calculating the closest distance reached by the line segment between and
*    two pairs of sequential points, and cuts out anything between two points
*    when their line segments get close. This algorithm will never compare two
*    consecutive line segments. Obviously the segments (p1,p2) and (p2,p3) will
*    get very close (they touch), but there would be nothing to trim between them.
*    The simplification step uses the Ramer-Douglas-Peucker algorithm.
*    See Wikipedia for description.
*
*    The simplification and pruning algorithms do not alter the path in memory,
*    and are designed to run in the background. They each have a parameter that
*    decides how long they will run before saving their state and returning. They
*    are both anytime algorithms, which is helpful when memory is filling up and
*    we just need to quickly identify a handful of points which can be deleted.
*    The algorithms can also report if they are done. If not, the thorough cleanup
*    will just have to wait for a bit until they are both done.
*/

RDP_Stack::RDP_Stack()
{
    stack = new start_finish[RDP_STACK_LEN];
    top = stack;
}

void RDP_Stack::push(start_finish item)
{
    *(++top) = item;
}

start_finish RDP_Stack::pop()
{
    return *(top--);
}

bool RDP_Stack::empty()
{
    return stack == top;
}

// TODO deal with issues that might arise from multithreading
Path::Path() :
    _last_index(0),
    _simplification_complete(False),
    _pruning_complete(False),
    _pruning_current_i(0),
    _pruning_max_j(0)
{
    _simplification_bitmask = std::bitset<MAX_PATH_LEN>().set(); //initialize to 1111...
    path = new Vector3f[MAX_PATH_LEN];
    _simplification_stack = new RDP_Stack();
}

void Path::append_if_far_enough(Vector3f p) {
    if (HYPOT(p, path[_last_index]) > POSITION_DELTA) {
        path[_last_index++] = p;
    }
}

void Path::routine_cleanup() {
    // We only do a routine cleanup if the memory is almost full. Cleanup deletes
    // points which are potentially useful, so it would be bad to clean up if we don't have to
    if (_last_index < MAX_PATH_LEN - 10) {
        return;
    }

    // TODO all of this:
    // retrieve the current cleanup_bitmask generated by _rdp() and
    // the std::vector of loops generated by _detect_loops()

    // if simplifying would remove more than 10, apply it.
    // else if its possible to prune more than 10, prune loops until 10+ have been removed
    // else if both together can remove 10+, do that (by running thorough_cleanup)
    // else give up, cleanly

    // end by resetting the state of the cleanup methods.
}

/**
*  Run this method only when preparing to initiate the RTL procedure. Returns a
*  pointer to the cleaned-up path. Returns nullptr if the cleanup algorithms aren'nt ready yet.
*  If this happens, just run this method again a bit later.
*/
void Path::thorough_cleanup() {
    // TODO all of this:
    // retrieve the current cleanup_bitmask generated by _rdp() and
    // the std::vector of loops generated by _detect_loops()

    // if ready flags are not set
    return nullptr;

    // Zero each Vector3f which should be removed.
    // Add back the halfway_points.

    // call the remove method.
    return &path;
}

//
// Private methods
//

/**
*    Simplifies a 3D path, according to the Ramer-Douglas-Peucker algorithm.
*    Returns the number of items which were removed. end_index is the index of the last element in the path.
*
*    This function has been tested. It produces the correct output.
*    TODO this function should return a bool, and leave a bitset in memory
*/
int Path::_rdp(uint8_t end_index, float epsilon)
{
    uint8_t start_index;
    uint8_t end_of_array = end_index;
    start_finish sf = {0, end_index};
    _simplification_stack->push(sf);
    while (!_simplification_stack->empty()) {
        start_finish tmp = _simplification_stack->pop();
        start_index = tmp.start;
        end_index = tmp.finish;

        float max_dist = 0.0f;
        uint8_t index = start_index;
        for (int i = index + 1; i < end_index; i++) {
            if (_simplification_bitmask[i]) {
                float dist = _point_line_dist(path[i], path[start_index], path[end_index]);
                if (dist > max_dist) {
                    index = i;
                    max_dist = dist;
                }
            }
        }

        if (max_dist > epsilon) {
            _simplification_stack->push(start_finish{start_index, index});
            _simplification_stack->push(start_finish{index, end_index});
        }
        else {
            for (int i = start_index + 1; i < end_index; i++) {
                _simplification_bitmask[i] = false;
            }
        }
    }
    return 0;
}

// TODO make this an anytime algorithm. Make is accept a time to run before pausing.
// then save the state somewhere. What to return? The state, a finished bool?
bool Path::_detect_loops() {
    // pruning step
    bool pruning_occured = FALSE;
    for (int i = 0; i <= _last_index; i++) {
        for (int j = _last_index - 1; j > i + 1; j--) {
            dist_point dp = _segment_segment_dist(path[i], path[i+1], path[j], path[j+1]);
            if (dp.distance <= PRUNING_DELTA) {
                // TODO return a struct (to-be-defined)
                pruning_occured = true;
            }
        }
    }
}

/**
* Removes all (0,0,0) points from the path, and shifts remaining items to correct position.
*/
int Path::_remove_empty_points() {
    int i = 0;
    int j = 0;
    int removed = 0;
    while (++i < _last_index) { // never removes the first item. This should obviously be {0,0,0}
        // if (path[i] == Vector3f(0.0f, 0.0f, 0.0f)) { // does this comparison work?
        if (path[i] == NULL){
            path[++j] = path[i]
        } else {
            removed++;
        }
    }
    _last_index -= removed;
}

void Path::_zero_points_by_bitmask() {
    for (int i = 0; i < end_of_array; i++){
        if (bitmask[i]) {
            // path[i] = Vector3f(0.0f, 0.0f, 0.0f);
            path[i] = NULL;
        }
    }
}

/**
*  Returns the closest distance in 3D space between any part of two input segments, defined from p1 to p2 and from p3 to p4.
*  Also returns the point which is halfway between
*
*  Limitation: This function does not work for parallel lines. In this case, it will return FLT_MAX. This does not matter for the path cleanup algorithm because
*  the pruning will still occur fine between the first parallel segment and a segment which is directly before or after the second segment.
*/
dist_point Path::_segment_segment_dist(Vector3f p1, Vector3f p2, Vector3f p3, Vector3f p4) {
    Vector3f u = p2-p1;
    Vector3f v = p4-p3;
    Vector3f w = p1-p3;

    float a = u*u;
    float b = u*v;
    float c = v*v;
    float d = u*w;
    float e = v*w;

    // the parameter for the position on line1 and line2 which define the closest points.
    float t1 = 0.0f;
    float t2 = 0.0f;

    if ((a*c)-(b*b) < SMALL_FLOAT) { // almost parallel. This avoids division by 0.
        return {FLT_MAX, Vector3f(0.0f, 0.0f, 0.0f)};
    }
    else {
        t1 = (b*e-c*d)/(a*c-b*b);
        t2 = (a*e-b*d)/(a*c-b*b);

        // restrict both parameters between 0 and 1.
        t1 = constrain_float(t1, 0.0f, 1.0f);
        t2 = constrain_float(t2, 0.0f, 1.0f);

        // difference between two closest points
        Vector3f dP = w+u*t1-v*t2;

        Vector3f halfway_point = (p1+u*t1 + p3+v*t2)/2.0f;
        return {dP.length(), halfway_point};
    }
}

/**
*  Returns the closest distance from a point to a 3D line. The line is defined by any 2 points
*/
float Path::_point_line_dist(Vector3f point, Vector3f line1, Vector3f line2) {
    // triangle side lengths
    float a = HYPOT(point, line1);
    float b = HYPOT(line1, line2);
    float c = HYPOT(line2, point);

    // semiperimeter of triangle
    float s = (a+b+c)/2.0f;

    float area_squared = s*(s-a)*(s-b)*(s-c);
    // must be constrained above 0 because a triangle where all 3 points could be on a line. float rounding could push this under 0.
    if (area_squared < 0.0f){
        area_squared = 0.0f;
    }
    float area = sqrt(area_squared);
    return 2.0f*area/b;
}

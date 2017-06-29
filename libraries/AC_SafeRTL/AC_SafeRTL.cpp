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

// TODO deal with issues that might arise from multithreading
Path::Path() :
    accepting_new_points(true),
    _last_index(0),
    _simplification_complete(false),
    _pruning_complete(false),
    _pruning_current_i(0),
    _pruning_min_j(0)
{
    _simplification_bitmask = std::bitset<MAX_PATH_LEN>().set(); //initialize to 1111...
    path[_last_index] = {0.0f, 0.0f, 0.0f};
}

void Path::append_if_far_enough(Vector3f p)
{
    if (!accepting_new_points) {
        return;
    }
    if (HYPOT(p, path[_last_index]) > POSITION_DELTA) {
        path[++_last_index] = p;
        // if cleanup algorithms are finished (And therefore not runnning), reset them

        // TODO refactor these out. also in the cleanups.
        if (_pruning_complete) {
            _pruning_complete = false;
            _pruning_current_i = 0;
            _pruning_min_j = 0;
        }
        if (_simplification_complete) {
            _simplification_complete = false;
            _simplification_bitmask.set();
            _simplification_stack.clear();
        }
    }
}

/**
*   Run this regularly, in the main loop (don't worry - it runs quickly). If no cleanup is needed, it will immediately return.
*   Otherwise, it will run a cleanup, based on info computed by the background methods, _rdp() and _detect_loops().
*   If no cleanup is possible, this method returns false. This should be treated as an error condition.
*/
bool Path::routine_cleanup()
{
    // We only do a routine cleanup if the memory is almost full. Cleanup deletes
    // points which are potentially useful, so it would be bad to clean up if we don't have to
    if (_last_index < MAX_PATH_LEN - 10) {
        return true;
    }

    bool success = false;

    int potential_amount_to_simplify = _simplification_bitmask.size() - _simplification_bitmask.count();

    // if simplifying will remove more than 10 points, just do it (tm)
    if (potential_amount_to_simplify > 10) {
        _zero_points_by_simplification_bitmask();
        _remove_empty_points();
        success = true;
    }

    // otherwise we'll see how much we could clean up by pruning loops
    int potential_amount_to_prune = 0;
    for (int i = 0; i < _prunable_loops.size() - 1; i++) {
        // consider that loops can overlap
        // This works because input loops are sorted chronologically (by start_index) and no loops are contained within other loops
        int end = _prunable_loops[i].end_index;
        if (_prunable_loops[i+1].start_index < end) {
            end = _prunable_loops[i+1].start_index;
        }
        potential_amount_to_prune += end - _prunable_loops[i].start_index;
    }
    // add last loop
    potential_amount_to_prune += _prunable_loops.back().end_index - _prunable_loops.back().start_index;
    // remove all of the halfway points that are going to get added back
    potential_amount_to_prune -= _prunable_loops.size();

    // if pruning could remove more than 10 points, prune loops until 10 or more points have been removed (doesn't necessarily prune all loops)
    if (potential_amount_to_prune > 10) {
        _zero_points_by_loops(10);
        _remove_empty_points();
        success = true;
    }

    // as a last resort, see if pruning and simplifying would remove 5+ points.
    if (potential_amount_to_prune + potential_amount_to_simplify) {
        _zero_points_by_simplification_bitmask();
        _zero_points_by_loops(10);
        _remove_empty_points();
        success = true;
    }

    // end by resetting the state of the cleanup methods.
    _simplification_complete = false;
    _simplification_stack.clear();
    _simplification_bitmask.set();

    _pruning_complete = false;
    _pruning_current_i = 0;
    _pruning_min_j = 0;
    _prunable_loops.clear();

    return success;
}

/**
*  Run this method only when preparing to initiate the RTL procedure. Returns a
*  pointer to the cleaned-up path. Returns nullptr if the cleanup algorithms aren't ready yet.
*  If this happens, just run this method again a bit later.
*/
Vector3f* Path::thorough_cleanup()
{
    if (!(_simplification_complete && _pruning_complete)) {
        return nullptr; // fail if the required cleanup data is not available
    }

    // apply simplification
    _zero_points_by_simplification_bitmask();

    // apply pruning
    _zero_points_by_loops(MAX_PATH_LEN); // prune every single loop

    _remove_empty_points();

    // end by resetting the state of the cleanup methods.
    _simplification_complete = false;
    _simplification_stack.clear();
    _simplification_bitmask.set();

    _pruning_complete = false;
    _pruning_current_i = 0;
    _pruning_min_j = 0;
    _prunable_loops.clear();

    return path;
}

Vector3f Path::get(int index)
{
    return path[index];
}

bool Path::cleanup_ready(){
    return _pruning_complete && _simplification_complete;
}

/**
*    Simplifies a 3D path, according to the Ramer-Douglas-Peucker algorithm.
*    Returns the number of items which were removed. end_index is the index of the last element in the path.
*/
void Path::rdp(uint32_t allowed_microseconds)
{
    if (_simplification_complete) {
        return;
    }
    else if (_simplification_stack.is_empty()) {  // if not complete but also nothing to do, we must be restarting.
        // reset to beginning state
        _simplification_stack.push_back(start_finish {0, _last_index});
    }
    uint32_t start_time = AP_HAL::micros();
    uint8_t start_index, end_index;
    while (!_simplification_stack.is_empty()) {
        if (AP_HAL::micros() - start_time > allowed_microseconds) {
            return;
        }

        start_finish tmp {}; // initialize to zero to suppress warnings
        _simplification_stack.pop_front(tmp);
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

        if (max_dist > RDP_EPSILON) {
            _simplification_stack.push_back(start_finish{start_index, index});
            _simplification_stack.push_back(start_finish{index, end_index});
        }
        else {
            for (int i = start_index + 1; i < end_index; i++) {
                _simplification_bitmask[i] = false;
            }
        }
    }
    _simplification_complete = true;
}

/**
*   This method runs for the allotted time, and detects loops in a path. All detected loops are added to _prunable_loops,
*   this function does not alter the path in memory.
*
*   Note that this method might take a bit longer than allowed_microseconds. It only stops after it's already run longer than allowed_microseconds.
*/
void Path::detect_loops(uint32_t allowed_microseconds)
{
    if (_pruning_complete) {
        return;
    }
    uint32_t start_time = AP_HAL::micros();

    while (_pruning_current_i < _last_index - 1) {
        if (AP_HAL::micros() - start_time > allowed_microseconds) {
            return;
        }

        uint8_t j = _pruning_current_i + 2;
        if (_pruning_min_j > j) {
            j = _pruning_min_j;
        }
        while (j < _last_index) {
            dist_point dp = _segment_segment_dist(path[_pruning_current_i], path[_pruning_current_i+1], path[j], path[j+1]);
            if (dp.distance <= PRUNING_DELTA) {
                _pruning_min_j = j;
                // int promotion rules prevent me from using i+1 and j+1
                _prunable_loops.push_back({(++_pruning_current_i)--,(++j)--,dp.point});
            }
            j++;
        }
        _pruning_current_i++;
    }
    _pruning_complete = true;
}

//
// Private methods
//

void Path::_zero_points_by_simplification_bitmask()
{
    for (int i = 0; i <= _last_index; i++) {
        if (_simplification_bitmask[i]) {
            path[i] = Vector3f(0.0f, 0.0f, 0.0f);
        }
    }
}

/**
*   Only prunes loops until points_to_delete points have been removed. It does not necessarily prune all loops.
*/
void Path::_zero_points_by_loops(uint8_t points_to_delete)
{
    int removed_points = 0;
    for (int i = 0; i < _prunable_loops.size(); i++) {
        loop l = _prunable_loops[i];
        for (int j = l.start_index; j < l.end_index; j++) {
            path[j] = Vector3f(0.0f, 0.0f, 0.0f);
        }
        path[int((l.start_index+l.end_index)/2.0)] = l.halfway_point;
        removed_points += l.end_index - l.start_index - 1;
        if (removed_points > points_to_delete) {
            return;
        }
    }
}

/**
* Removes all NULL points from the path, and shifts remaining items to correct position.
*/
void Path::_remove_empty_points()
{
    int i = 0;
    int j = 0;
    int removed = 0;
    while (++i < _last_index) { // never removes the first item. This should obviously be {0,0,0}
        if (path[i] == Vector3f(0.0f, 0.0f, 0.0f)) {
            path[++j] = path[i];
        }
        else {
            removed++;
        }
    }
    _last_index -= removed;
}

/**
*  Returns the closest distance in 3D space between any part of two input segments, defined from p1 to p2 and from p3 to p4.
*  Also returns the point which is halfway between
*
*  Limitation: This function does not work for parallel lines. In this case, it will return FLT_MAX. This does not matter for the path cleanup algorithm because
*  the pruning will still occur fine between the first parallel segment and a segment which is directly before or after the second segment.
*/
// typedef struct dist_point dist_point;
Path::dist_point Path::_segment_segment_dist(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3, const Vector3f &p4)
{
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
float Path::_point_line_dist(const Vector3f &point, const Vector3f &line1, const Vector3f &line2)
{
    // triangle side lengths
    float a = HYPOT(point, line1);
    float b = HYPOT(line1, line2);
    float c = HYPOT(line2, point);

    // semiperimeter of triangle
    float s = (a+b+c)/2.0f;

    float area_squared = s*(s-a)*(s-b)*(s-c);
    // must be constrained above 0 because a triangle where all 3 points could be on a line. float rounding could push this under 0.
    if (area_squared < 0.0f) {
        area_squared = 0.0f;
    }
    float area = sqrt(area_squared);
    return 2.0f*area/b;
}

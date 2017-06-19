#include "AC_SafeRTL.h"


RDP_Stack::RDP_Stack()
{
    stack = new start_finish[RDP_STACK_LEN];
    top = &stack;
}

RDP_Stack::push(start_finish *item)
{
    stack[++top] = *item;
}

RDP_Stack::pop()
{
    return stack[top--];
}

RDP_Stack::empty()
{
    return stack == top;
}

Path::Path()
{
    path = new Vector3f[MAX_PATH_LEN];
    last_index = 0;
    worst_length = 0;
    RDP_Stack * stack = new RDP_Stack();
}

/**
*   Simplifies a 3D path, according to the Ramer-Douglas-Peucker algorithm.
*   Returns the number of items which were removed.
*/
int Path::rdp(Vector3f *path, int start_index, int end_index, float epsilon)
{
    start_finish sf = {start_index, end_index};
    stack->push(sf);
    global_start = start;
    // The bitmask which represents which points to keep (1) and which to delete(0)
    std::bitset<end_index-start_index> bitmask = bitset<end_index-start_index>().set(); //initialized to 1
    bitmask.set();
    while (!stack.empty()) {
        start_finish tmp = stack.pop();
        start_index = tmp->start;
        end_index = tmp->finish;

        float max_dist = 0f;
        int index = start_index;
        for (int i = index + 1; i < end_index; i++) {
            if (bitmask[i-global_start]) {
                float dist = point_line_dist(path[i], path[start_index], path[end_index]);
                if (dist > dist_max) {
                    index = i;
                    max_dist = dist;
                }
            }
        }

        if (dmax > epsilon) {
            stack.push(start_finish{start_index, index});
            stack.push(start_finish{index, end_index});
        }
        else {
            for (int i = startIndex + 1; i < lastIndex; ++i) {
                bitmask[i-global_start] = false;
            }
        }
    }
    // in-place removal of objects from the array based on the bitset object.
    int i = start_index;
    int j = i;

    int removed = 0; // counts number of items removed from list
    while (++i < end_index){
        if (bitmask[i]){ //keep this item
            path[++j] = path[i];
        } else {
            removed++;
        }
    }
    return removed;
}

void Path::append_if_far_enough(Vector3f p) {
    if (HYPOT(p, path[last_index]) > position_delta) {
        path[last_index++] = p;
    }
}

void Path::routine_cleanup() {
    // We only do a routine cleanup if the memory is almost full. Cleanup deletes potentially useful points,
    // so it would be bad to clean up if we don't have to
    if (last_index > MAX_PATH_LEN - 2) {
        Path::cleanup();
        if (last_index > MAX_PATH_LEN - 2) { // if cleanup was unsuccesful
            ;// TODO crap out, cleanly
        }
    }
}

/**
*  Run this method only when preparing to initiate the RTL procedure.
*/
void Path::thorough_cleanup() {
    while (Path::cleanup());
}

/**
*    Takes a path and runs 2 cleanup steps: pruning, then simplification.
*
*    The pruning step defines line segments from point 1 to 2, point 2 to 3, ...
*    Then it compares (almost) all line segments to see how close they got, in 3D space.
*    If they got close enough, defined by the parameter 'position_delta', all path points
*    between those two line segments are deleted, and replaced by a single point halfway between
*    where the two previous line segments were closest.
*
*    This algorithm will never compare two consecutive line segments. Obviously
*    the segments (p1,p2) and (p2,p3) will get very close (they touch), but there would be nothing to trim between them.
*
*    If the deletion is triggered, the pruning step is complete. Since certain line segments are now gone,
*    it does not make sense to keep comparing using those (potentially deleted) line segments. The goal of this algorithm
*    is not to find the optimal simplified path, but rather to simplify it enough that it is not at risk of running out of memory.
*
*    The simplification step uses the Ramer-Douglas-Peucker algorithm. See Wikipedia for description.
*
*    Returns true if pruning occured. In this case, running cleanup() again might prune even more.
*    But if no pruning occured (returns False) then running the algorithm again will change nothing.
*/
bool Path::cleanup() {
    // pruning step
    bool pruning_occured = FALSE;
    for (int i = 0; i <= last_index; i++) {
        for (int j = last_index - 1; j > i + 1; j--) {
            distance_point dp = segment_segment_dist(path[i], path[i+1], path[j], path[j+1]);
            if (dp->distance <= PRUNING_DELTA) {
                // TODO prune path. Not ideal with an array. Looks like this in python: self.path = self.path[:i+1] + [dist[1]] + self.path[j+1:]
                // probably do this with memset or maybe std::move
                pruning_occured = True;
                goto simplification_step; // best way I could think of to break out of nested loops. sorry.
            }
        }
    }
    simplification_step:
    rdp(Vector3f *path, last_index, RDP_EPSILON);

    return pruning_occured;
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
    float t1 = 0f;
    float t2 = 0f;

    if ((a*c)-(b*b) < SMALL_FLOAT) { // almost parallel. This avoids division by 0.
        return {FLT_MAX, Vector3f(0, 0, 0)};
    }
    else {
        t1 = (b*e-c*d)/(a*c-b*b);
        t2 = (a*e-b*d)/(a*c-b*b);

        // restrict both parameters between 0 and 1.
        t1 = min(max(0,t1),1);
        t2 = min(max(0,t2),1);

        // difference between two closest points
        Vector3f dP = w+t1*u-t2*v;

        halfway_point = (p1+t1*u + p3+t2*v)/2.;
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
    float s = (a+b+c)/2f;

    area = sqrt(max(0f,s*(s-a)*(s-b)*(s-c))); //inner part must be constrained above 0 because a triangle where all 3 points could be on a line. float rounding could push this under 0.
    return 2*area/b;
}

#include <vector>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AC_SafeRTL/AC_SafeRTL.h>

// vectors defined below:
// test_path_before
// test_path_after_adding
// test_path_after_simplifying
// test_path_after_pruning
// test_path_complete

// assume that any point without a comment should be kept
std::vector<Vector3f> test_path_before {
    {0.0, 0.0, 0.0},
    {3.0, 0.0, 0.0}, // simplified
    {3.0, 0.0, 0.0}, // not added
    {6.0, 0.0, 0.0}, // simplified
    {10.0, 0.0, 0.0},
    {10.0, 3.0, 0.0},
    {13.0, 3.0, 0.0},
    {13.0, 6.0, 0.0},
    {16.0, 6.0, 0.0},
    {16.0, 6.0, 1.0}, // not added
    {16.0, 8.0, 1.0},
    {18.0, 8.0, 0.0},
    {20.0, 10.0, 0.0},
};

std::vector<Vector3f> test_path_after_adding {
    {0.0, 0.0, 0.0},
    {3.0, 0.0, 0.0}, // simplified
    {6.0, 0.0, 0.0}, // simplified
    {10.0, 0.0, 0.0},
    {10.0, 3.0, 0.0},
    {13.0, 3.0, 0.0},
    {13.0, 6.0, 0.0},
    {16.0, 6.0, 0.0},
    {16.0, 8.0, 1.0},
    {18.0, 8.0, 0.0},
    {20.0, 10.0, 0.0},
};

std::vector<Vector3f> test_path_after_simplifying {
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {10.0, 3.0, 0.0},
    {13.0, 3.0, 0.0},
    {13.0, 6.0, 0.0},
    {16.0, 6.0, 0.0},
    {16.0, 8.0, 1.0},
    {18.0, 8.0, 0.0},
    {20.0, 10.0, 0.0},
};

std::vector<Vector3f> test_path_after_pruning {};

std::vector<Vector3f> test_path_complete {};

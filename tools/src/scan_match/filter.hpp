#pragma once
#include <iostream>
#include <scan_match/var_defs.hpp>
#include <ukf_state_msg/State.h>
#include <scan_match/utils.hpp>

using namespace std;
using namespace ukf_state_msg;

class Filter
{

public:
    Filter() = default;
    MyPointCloud2D getScanPointsWithinThreshold(MyPointCloud2D scans);
    Matrix2d allignScanPoints(MyPointCloud2D &scans, const State &state);
    MyPointCloud2D reduceMap(MyPointCloud2D map_carpark, const State &state);

private:
    const float longi_max = 40;
    const float longi_min = -40;
    const float lateral_max = 35;
    const float lateral_min = -35;
    const float map_threshold = 40;
};

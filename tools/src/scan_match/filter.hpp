#pragma once
#include <iostream>
#include <scan_match/var_defs.hpp>
#include <ukf_state_msg/State.h>
#include <scan_match/utils.hpp>
#include <Model_Development_bridge/autobox_out.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;
using namespace ukf_state_msg;
using namespace Model_Development_bridge;
using namespace sensor_msgs;
using namespace geometry_msgs;

class Filter
{

public:
    Filter() = default;
    MyPointCloud2D getScanPointsWithinThreshold(MyPointCloud2D scans);
    Matrix2d allignScanPoints(MyPointCloud2D &scans, const State &state);
    MyPointCloud2D reduceMap(MyPointCloud2D map_carpark, const State &state);
    sensor_msgs::PointCloud filterLaserChannel(sensor_msgs::PointCloud pc);

public:
    float longi_max = 75;
    float longi_min = -75;
    float lateral_max = 75;
    float lateral_min = -75;
    float map_threshold = 75;
};

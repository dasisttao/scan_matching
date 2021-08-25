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
    MyPointCloud2D getScanPointsWithinThreshold_param(MyPointCloud2D scans, float longiMax, float longiMin, float lateralMax, float lateralMIN);
    MyPointCloud2D getScanPointsWithinThreshold_param(MyPointCloud2D scans, float longiMax, float longiMin, float lateralMax1, float lateralMIN1, float lateralMax2, float lateralMIN2);
    Matrix2d allignScanPoints(MyPointCloud2D &scans, const State &state);
    MyPointCloud2D reduceMap(MyPointCloud2D map_carpark, const State &state);
    sensor_msgs::PointCloud filterLaserChannel(sensor_msgs::PointCloud pc);
    bool is_this_frame_has_4to7Layer_PCL = true;
public:
    float longi_max = 85;
    float longi_min = -85;
    float lateral_max = 85;
    float lateral_min = -85;
    float map_threshold = 85;
};

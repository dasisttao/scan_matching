#pragma once
#include <iostream>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <scan_match/var_defs.hpp>
#include <scan_match/transform.hpp>


using namespace std;
class Timer
{
public:
    void start();
    double stop();

private:
    std::chrono::_V2::system_clock::time_point beg;
    std::chrono::_V2::system_clock::time_point end;
};

class RVIZ
{
public:
    sensor_msgs::PointCloud2 createPointCloud(MyPointCloud2D my_pc, string frame_id, float height);
    void outputLocalizationResult(const State state, geometry_msgs::PoseStamped &result_output);
    void displayEstimatedPose(const State state, geometry_msgs::PoseStamped &pose_estimation);

private:
    CoordTransform coord_transform;
};

class ScanPoints
{
public:
    MyPointCloud2D create(sensor_msgs::PointCloud msg);
};

class Ramp
{
public:
    vector<bool> check(const State state);

private:
    bool entered_ramp = false;
    bool entered_ramp2 = false;
};

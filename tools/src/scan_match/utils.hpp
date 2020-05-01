#pragma once
#include <iostream>
#include <chrono>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <scan_match/var_defs.hpp>

using namespace std;
class Timer
{
public:
    void start();
    void stop(string msg);

private:
    std::chrono::_V2::system_clock::time_point beg;
    std::chrono::_V2::system_clock::time_point end;
};

class RVIZ
{
public:
    sensor_msgs::PointCloud2 createPointCloud(MyPointCloud2D my_pc, string frame_id);
};
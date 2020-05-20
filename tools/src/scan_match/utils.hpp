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
    double stop();

private:
    std::chrono::_V2::system_clock::time_point beg;
    std::chrono::_V2::system_clock::time_point end;
};

class RVIZ
{
public:
    sensor_msgs::PointCloud2 createPointCloud(MyPointCloud2D my_pc, string frame_id, float height);
};

// vector<double> utm_local_pt1;
//   vector<double> utm_local_pt2;
//   vector<double> utm_local_pt3;
//   vector<double> utm_local_pt4;
//   vector<double> pt1{606167.51, 5797146.69, 0};
//   vector<double> pt2{606198.75, 5797156.54, 0};
//   vector<double> pt3{606192.28, 5797068.07, 0};
//   vector<double> pt4{606223.55, 5797077.92, 0};
//   utm_local_pt1 = ukfnode.UTM2Local(pt1);
//   utm_local_pt2 = ukfnode.UTM2Local(pt2);
//   utm_local_pt3 = ukfnode.UTM2Local(pt3);
//   utm_local_pt4 = ukfnode.UTM2Local(pt4);
//   MyPointCloud2D scans;
//   MyPoint temp_pt;
//   temp_pt.x = utm_local_pt1[0];
//   temp_pt.y = utm_local_pt1[1];
//   scans.pts.push_back(temp_pt);
//   temp_pt.x = utm_local_pt2[0];
//   temp_pt.y = utm_local_pt2[1];
//   scans.pts.push_back(temp_pt);
//   temp_pt.x = utm_local_pt3[0];
//   temp_pt.y = utm_local_pt3[1];
//   scans.pts.push_back(temp_pt);
//   temp_pt.x = utm_local_pt4[0];
//   temp_pt.y = utm_local_pt4[1];
//   scans.pts.push_back(temp_pt);
//   pc2 = rviz.createPointCloud(scans, "ibeo_lux", 2);

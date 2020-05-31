#pragma once
#include "ros_can_gps_msg/gpsData.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <scan_match/var_defs.hpp>
#include <ros_can_gps_msg/gpsData.h>

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace ros_can_gps_msg;

class CoordTransform
{
public:
	std::vector<double> Local2UTM(const std::vector<double> &sv);
	// @Transform from UTM coordinate to Local
	std::vector<double> UTM2Local(const std::vector<double> &sv);
	// @Transform wgs84 coordinates to utm
	bool wgs84deg2utm(const std::vector<double> &deg, std::vector<double> &utm);
	// @Transform wgs84 orientation to utm
	double wgs84yaw2utm(const std::vector<double> &deg);
	// @Transform wgs84 coordinates and orientation to utm
	bool wgs84Toutm(const std::vector<double> &deg, std::vector<double> &utm);
	// @Transform from wgs84 to utm to local for kalman
	vector<double> getLocalPoseFromGPS(const gpsData::ConstPtr &gps_data, geometry_msgs::PoseStamped &gps_pose);
};

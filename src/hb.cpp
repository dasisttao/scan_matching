#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <ukf_state_msg/State.h>
#include <ukf/ukf.h>
#include <map/map.hpp>
#include <geodesy/utm.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Model_Development_bridge/autobox_out.h>

//http://wiki.ros.org/message_filters#ExactTime_Policy
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros_can_gps_msg/gpsData.h>

#include <scan_match/var_defs.hpp>
#include <scan_match/filter.hpp>
#include <scan_match/icp.hpp>
#include <scan_match/utils.hpp>
#include <csv/csv.h>
#include <csv/csv_main.h>
#include <scan_match/transform.hpp>
#include <debugging/plausability.h>

#include "myInit.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace ukf_state_msg;
using namespace ros_can_gps_msg;
using namespace Model_Development_bridge;
using namespace WriteCSV;
using namespace ReadCSV;

//Variablen
bool init = false;

Filter filter;
Timer timer;
ICP icp;

Ramp ramp;

Plausability plausability;
CoordTransform coord_transform;




double time_us_;
double time_start = 0;

geometry_msgs::PoseStamped result_output;

enum MeasureState
{
  Odo,
  Laser,
  GPS
};
MeasureState measure_state;

MyPointCloud2D createMyPointCloud(sensor_msgs::PointCloud msg)
{
  MyPointCloud2D scan_points;
  MyPoint temp_pt;
  for (int i = 0; i < msg.points.size(); ++i)
  {
    scan_points.ids.push_back(i);
    scan_points.weights.push_back(1);
    scan_points.distances.push_back(0);
    temp_pt.x = msg.points[i].x;
    temp_pt.y = msg.points[i].y;
    scan_points.pts.push_back(temp_pt);
  }
  return scan_points;
}


void callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped &pose_in)
{
  tf::Quaternion q(
      pose_in.pose.pose.orientation.x,
      pose_in.pose.pose.orientation.y,
      pose_in.pose.pose.orientation.z,
      pose_in.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ukf_filter.x_ << pose_in.pose.pose.position.x, pose_in.pose.pose.position.y, 0, yaw, 0;
  ukf_filter.P_ << 10, 0.03, -0.014, 0, 0,
      0.03, 10, -0.01, 0, 0,
      -0.014, 0, 0.05, 0, 0,
      0, 0, 0, 0, 0,
      0, 0, 0, 0, 0;
  std::cout <<  " Initialized X:   "<<  pose_in.pose.pose.position.x << std::endl;
  std::cout <<  " Initialized Y:   "<< pose_in.pose.pose.position.y << std::endl;
  std::cout <<  " Initialized Yaw: "<< yaw << std::endl;

  state.x = ukf_filter.x_(0);
  state.y = ukf_filter.x_(1);
  state.v = ukf_filter.x_(2);
  state.yaw = ukf_filter.x_(3);
  state.yawr = ukf_filter.x_(4);
  //Display estimated pose on rviz (here it is inital pose)
  rviz.displayEstimatedPose(state, pose_estimation);
  initial_pose_needed = false;
}

void callback(const PointCloud2::ConstPtr &point_cloud, const autobox_out::ConstPtr &can_data)
{

  double tnow = point_cloud->header.stamp.toSec();
  double dt = tnow - ukf_filter.time_us_;
  if (!init)
  {
    init = true;
    ukf_filter.time_us_ = point_cloud->header.stamp.toSec();
    time_start = point_cloud->header.stamp.toSec();

    measure_state = MeasureState::Odo;
    return;
  }
  //Return (not use Algorithm) if initial pose is needed (e.g. because started in parkhaus)
  if(initial_pose_needed == true){
	ukf_filter.time_us_ = tnow;
  return;
  }
  //Save last state in csv
  WriteCSV::kalmanCSV(ukf_filter);
  //Setup current state for ICP Algorithm
  state.x = ukf_filter.x_(0);
  state.y = ukf_filter.x_(1);
  state.v = ukf_filter.x_(2);
  state.yaw = ukf_filter.x_(3);
  state.yawr = ukf_filter.x_(4);

  //Plausability
  plausability.setState(state);

  //Display estimated pose on rviz
  rviz.displayEstimatedPose(state, pose_estimation);
  //Output node localization result
  rviz.outputLocalizationResult(state, result_output);

  //Use localization method depending on GPS signal quality
    vector<bool> on_ramp = ramp.check(state);
    if (measure_state == MeasureState::Laser)
    {
      ukf_filter.Prediction(dt);

      //Transform Pointcloud2 into Pointcloud
      sensor_msgs::PointCloud pc;
      sensor_msgs::convertPointCloud2ToPointCloud(*point_cloud, pc);

      //Filter roof scans
      pc = filter.filterLaserChannel(pc);

      //Create ScanPoints vector for better handling
      MyPointCloud2D scans = createMyPointCloud(pc);

      //------Algorithm Start-------
      //--1--Reducing ScanPoints
      scans = filter.getScanPointsWithinThreshold(scans);

      //--2--Pre-Allignment (Vorausrichtung)
      Matrix2d rotM = filter.allignScanPoints(scans, state);

      //--3--Reduce points in map
      MyPointCloud2D map_filt = filter.reduceMap(map_carpark, state);

      // //--4--ICP Algorithmen
      scans = icp.mainAlgorithm(map_filt, scans, state, icp_state, rotM);

      //Update UKF Laser
      ukf_filter.UpdateLaser(icp_state);

      //Plausability - First Odo, then Laser, because State is getting changed on Laser Update
      plausability.times.push_back((can_data->header.stamp.toSec() - time_start));
      plausability.setStateOdo(ukf_filter.UpdateOdometriePlausability(can_data, on_ramp));
      plausability.setStateICP(ukf_filter);

      //!!!For RVIZ !!! NOTE: comment this out on release versions to save resources
      map_pc = rviz.createPointCloud(map_filt, "ibeo_lux", 1.0);
      pc2 = rviz.createPointCloud(scans, "ibeo_lux", 1.1);

      //Switch to Odo
      measure_state = MeasureState::Odo;
    }
    else if (measure_state == MeasureState::Odo || measure_state == MeasureState::GPS)
    {
      ukf_filter.Prediction(dt);
      ukf_filter.UpdateOdometrie(can_data, on_ramp);
      measure_state = MeasureState::Laser;
    }

  //Plausability Checks
  plausability.deltaOdoICP();

  ukf_filter.time_us_ = tnow;
}

int main(int argc, char **argv)
{
  //Node Handling
  ros::init(argc, argv, "scan_matching_node");
  ros::NodeHandle nh;
  //Subs
  ros::Subscriber sub = nh.subscribe("/initialpose", 1000, callback_init_pose);
  //Sync sub msgs
  message_filters::Subscriber<PointCloud2> point_cloud_sub(nh, "/as_tx/point_cloud", 10);
  message_filters::Subscriber<autobox_out> can_sub(nh, "/data_out", 10);
  typedef sync_policies::ApproximateTime<PointCloud2, autobox_out> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_cloud_sub, can_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  auto posepubUTM = nh.advertise<geometry_msgs::PoseStamped>("VehiclePoseFusionUTM", 10);
  auto pc_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_point_cloud", 30);
  auto map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_point_cloud", 30);
  auto pose_est_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_estimated", 30);
  auto pose_test_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_test", 30);
  ros::Rate rate(60);

  MYINIT myinit(nh,rate,map_pub);

  while (ros::ok())
  {
    //UTM
    posepubUTM.publish(result_output);
    //Point Cloud Laser
    pc2.header.stamp = ros::Time::now();
    pc_pub.publish(pc2);
    //Point Cloud Map
    map_pc.header.stamp = ros::Time::now();
    map_pub.publish(map_pc);
    //Estimated Pose
    pose_estimation.header.stamp = ros::Time::now();
    pose_estimation.header.frame_id = "ibeo_lux";
    pose_est_pub.publish(pose_estimation);

    //Test pose
    pose_test_pub.publish(pose_test);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

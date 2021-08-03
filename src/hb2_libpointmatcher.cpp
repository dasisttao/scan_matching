#include <ros/ros.h>
#include <iostream>
#include <fstream>
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
#include "Floor.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <chrono>

#include <pcl/registration/ndt.h>

#include "pointmatcher/PointMatcher.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace ukf_state_msg;
using namespace ros_can_gps_msg;
using namespace Model_Development_bridge;
using namespace WriteCSV;
using namespace ReadCSV;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

int64_t MessZeit {0};
size_t CountMess {0};

std::vector<int64_t> MessZeit_V;

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

void saveMyPointCloudtoPCLXYZ(const MyPointCloud2D &mypointcloud2d, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
  // Fill in the CloudIn data
	cloud->width = mypointcloud2d.pts.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

  for (auto i = 0; i < mypointcloud2d.pts.size(); i++){
    cloud->points[i].x = mypointcloud2d.pts[i].x;
    cloud->points[i].y = mypointcloud2d.pts[i].y;
    cloud->points[i].z = 0;
  }
}

MyPointCloud2D savePCLXYZtoMyPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud){
  // Fill in the CloudIn data
  MyPointCloud2D mypointcloud2d;
  
  for (auto i = 0; i < cloud.points.size(); i++ ){

    MyPoint temp_pt;
    temp_pt.x = cloud.points[i].x;
    temp_pt.y = cloud.points[i].y;
    mypointcloud2d.pts.push_back(temp_pt);
    mypointcloud2d.ids.push_back(i);
  }
  return mypointcloud2d;
}

MyPointCloud2D savePCLXYZtoMyPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
  // Fill in the CloudIn data
  MyPointCloud2D mypointcloud2d;
  
  for (auto i = 0; i < cloud->points.size(); i++ ){

    MyPoint temp_pt;
    temp_pt.x = cloud->points[i].x;
    temp_pt.y = cloud->points[i].y;
    mypointcloud2d.pts.push_back(temp_pt);
    mypointcloud2d.ids.push_back(i);
  }
  return mypointcloud2d;
}


sensor_msgs::PointCloud2 MyPointCloud2D_to_Sensor_Stard_PointClooud2(MyPointCloud2D my_pc, float height)
{
    sensor_msgs::PointCloud pc;
    pc.points.resize(my_pc.pts.size());
    pc.header.stamp = ros::Time::now();
    sensor_msgs::ChannelFloat32 depth_channel;

    for (int i = 0; i < my_pc.pts.size(); i++)
    {
        pc.points[i].x = my_pc.pts[i].x;
        pc.points[i].y = my_pc.pts[i].y;
        pc.points[i].z = height;
        depth_channel.values.push_back(1);
    }
    pc.channels.push_back(depth_channel);
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
    return pc2;
}



State calcNewState(const State& state, const Eigen::Matrix4f &transform_M)
{
    State new_state;
    Vector2f new_pos;
    Vector2f pos;
    Matrix2f new_rot;
    pos << state.x, state.y;

    Matrix2f rot_M;
    rot_M << transform_M(0,0), transform_M(0,1), transform_M(1,0), transform_M(1,1);

    Vector2f trans_V;
    trans_V << transform_M(0,3), transform_M(1,3);

    new_pos = rot_M * pos + trans_V;

    new_state.x = new_pos(0);
    new_state.y = new_pos(1);
    new_state.v = state.v;
    new_state.yawr = state.yawr;


    new_state.yaw = atan2(rot_M(1,0), rot_M(0,0));
    new_state.yaw = state.yaw + new_state.yaw;

    return new_state;
}




void Vorausrichtung(sensor_msgs::PointCloud &pc, const State &state)
{
    Matrix2d rot_M;
    Vector2d ego_pos;
    Vector2d temp;
    Vector2d result;
    ego_pos << state.x, state.y;

    double alpha = (-state.yaw);
    rot_M
        << cos(alpha),
        sin(alpha),
        -sin(alpha), cos(alpha);
    for (int i = 0; i < pc.points.size(); i++)
    {
        temp << pc.points[i].x , pc.points[i].y;

        result = rot_M * temp + ego_pos;
        pc.points[i].x = result(0);
        pc.points[i].y = result(1);
        pc.points[i].z = 0.0;
    }
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
  // WriteCSV::kalmanCSV(ukf_filter);
  //Setup current state for ICP Algorithm
  state.x = ukf_filter.x_(0);
  state.y = ukf_filter.x_(1);
  state.v = ukf_filter.x_(2);
  state.yaw = ukf_filter.x_(3);
  state.yawr = ukf_filter.x_(4);

  //Plausability
  // plausability.setState(state);

  //Display estimated pose on rviz
  rviz.displayEstimatedPose(state, pose_estimation);
  //Output node localization result
  rviz.outputLocalizationResult(state, result_output);

  //Use localization method depending on GPS signal quality
    vector<bool> on_ramp = ramp.check(state);
    if (measure_state == MeasureState::Laser)
    {
      try
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
        //--0--Reducing ScanPoints
        scans = filter.getScanPointsWithinThreshold(scans);


        //--1--Pre-Allignment (Vorausrichtung) + change formate in order to use libpointmatcher
        Matrix2d rotM = filter.allignScanPoints(scans, state);
        pc2 = rviz.createPointCloud(scans, "ibeo_lux", 0);
        PointMatcher<float>::DataPoints data = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(pc2, false);


        //--2--Reduce points in map + change formate in order to use libpointmatcher
        MyPointCloud2D map_filt = filter.reduceMap(map_carpark, state);
        map_pc = rviz.createPointCloud(map_filt, "ibeo_lux", 0);
        PointMatcher<float>::DataPoints ref = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(map_pc, false);

        //--3--ICP using libpointmatcher with yaml to configure the filter

          // Get Filter to be used in ICP from yaml Config file
          PM::ICP icp;
          std::string configFile = "src/icp_lokalisierung/scan_matching/cfg/ICP_config.yaml"; 
          std::ifstream ifs(configFile.c_str());
          if (!ifs.good())
          {
            cerr << "Cannot open config file " << configFile << "\n"; exit(1);
          }
          icp.loadFromYaml(ifs);

          // Compute the transformation to express data in ref
          PM::TransformationParameters T = icp(data, ref);
          // Change the pointcloud form laser so it can be visualized 
          icp.transformations.apply(data, T);
          // Get the ICP state
          icp_state = calcNewState(state,T);
          // update pc2 to visulize the changed pointcloud
          pc2 = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(data, "ibeo_lux", ros::Time::now());


        // UKF Update with ICP State  
        ukf_filter.UpdateLaser(icp_state);
   
        //Switch to Odo
        measure_state = MeasureState::Odo;
      }
      catch (std::exception const &exc)
      {
          std::cerr << "Exception caught " << exc.what() << "\n";
          return;
      }
    }
    else if (measure_state == MeasureState::Odo || measure_state == MeasureState::GPS)
    {
      ukf_filter.Prediction(dt);
      ukf_filter.UpdateOdometrie(can_data, on_ramp);
      measure_state = MeasureState::Laser;
    }

  //Plausability Checks
  // plausability.deltaOdoICP();

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
  // auto pose_test_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_test", 30);
  ros::Rate rate(60);

  MYINIT myinit(nh,rate,map_pub);
  FLOOR floor(nh, FLOOR::CarOnFloor::E0);
  floor.printFloor();
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

    floor.check_floor(state);
    //Test pose
    // pose_test_pub.publish(pose_test);
    ros::spinOnce();
    rate.sleep();
  }
  std::ofstream myfile;
      myfile.open ("example.csv");
      for (auto i=0; i< MessZeit_V.size(); i++){
        myfile << std::to_string(MessZeit_V[i]) << "\n";
      }
      myfile.close();
  return 0;
}

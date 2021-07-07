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
#include "Floor.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>  

Filter filter;

sensor_msgs::PointCloud2 offlineMap_to_vis;
sensor_msgs::PointCloud2 onlinePCL_to_vis;
sensor_msgs::PointCloud2 afterMatch_to_vis;

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


sensor_msgs::PointCloud2 VIS_PCL(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,string frame_id, float height)
{
    sensor_msgs::PointCloud pc;
    pc.points.resize(cloud->points.size());
    pc.header.frame_id = frame_id;
    pc.header.stamp = ros::Time::now();
    sensor_msgs::ChannelFloat32 depth_channel;

    for (int i = 0; i < cloud->points.size(); i++)
    {
        pc.points[i].x = cloud->points[i].x;
        pc.points[i].y = cloud->points[i].y;
        pc.points[i].z = height;
        depth_channel.values.push_back(1);
    }
    pc.channels.push_back(depth_channel);
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
    return pc2;
}

sensor_msgs::PointCloud2 VIS_PCL(const pcl::PointCloud<pcl::PointXYZ>& cloud,string frame_id, float height)
{
    sensor_msgs::PointCloud pc;
    pc.points.resize(cloud.points.size());
    pc.header.frame_id = frame_id;
    pc.header.stamp = ros::Time::now();
    sensor_msgs::ChannelFloat32 depth_channel;

    for (int i = 0; i < cloud.points.size(); i++)
    {
        pc.points[i].x = cloud.points[i].x;
        pc.points[i].y = cloud.points[i].y;
        pc.points[i].z = height;
        depth_channel.values.push_back(1);
    }
    pc.channels.push_back(depth_channel);
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
    return pc2;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);



void callback_pcl(const PointCloud2::ConstPtr &point_cloud){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);

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

  saveMyPointCloudtoPCLXYZ(scans,cloud_source);

  onlinePCL_to_vis = VIS_PCL(cloud_source,"ibeo_lux", 1.0);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);

  icp.setMaxCorrespondenceDistance (1);
  // // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (2);
  // // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (0.01);
  // // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);
  
  pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

  afterMatch_to_vis = VIS_PCL(Final,"ibeo_lux", 1.1);
   

}





int main(int argc, char **argv)
{
  //Node Handling
  ros::init(argc, argv, "scan_matching_node");
  ros::NodeHandle nh;
  //Subs
  ros::Subscriber sub_pcl = nh.subscribe("/as_tx/point_cloud", 10, callback_pcl);
  
  //Subs
  ros::Subscriber sub = nh.subscribe("/initialpose", 1000, callback_init_pose);
  auto posepubUTM = nh.advertise<geometry_msgs::PoseStamped>("VehiclePoseFusionUTM", 10);
  auto pc_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_point_cloud", 30);
  auto map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_point_cloud", 30);

  auto afterMatch = nh.advertise<sensor_msgs::PointCloud2>("afterMatch", 30);


  auto pose_est_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_estimated", 30);
  auto pose_test_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_test", 30);
  ros::Rate rate(60);

  my_map.readRawData(map_pc, map_carpark,"src/icp_lokalisierung/scan_matching/map/map_Elphi_R0_E1_Bauplan_reduced.csv");
  saveMyPointCloudtoPCLXYZ(map_carpark,cloud_target);
  offlineMap_to_vis = VIS_PCL(cloud_target,"ibeo_lux", 1.0);
  MYINIT myinit(nh,rate,map_pub);

  

  

  while (ros::ok())
  {
    
    // Point Cloud Laser
    onlinePCL_to_vis.header.stamp = ros::Time::now();
    pc_pub.publish(onlinePCL_to_vis);
    //Point Cloud Map
    offlineMap_to_vis.header.stamp = ros::Time::now();
    map_pub.publish(offlineMap_to_vis);


    //Point Cloud Map
    afterMatch_to_vis.header.stamp = ros::Time::now();
    afterMatch.publish(afterMatch_to_vis);


    //Estimated Pose
    pose_estimation.header.stamp = ros::Time::now();
    pose_estimation.header.frame_id = "ibeo_lux";
    pose_est_pub.publish(pose_estimation);

    // floor.check_floor(state);
    // //Test pose
    // pose_test_pub.publish(pose_test);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

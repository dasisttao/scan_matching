
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <ukf_state_msg/State.h>

#include <geodesy/utm.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <tf2/LinearMath/Quaternion.h>
#include <gps_transforms/transform.hpp>

#include <nav_msgs/OccupancyGrid.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace ukf_state_msg;
using namespace ros_can_gps_msg;

//Variablen
MyPointCloud2D map_carpark;
Filter filter;
Timer timer;
ICP icp;
RVIZ rviz;
UkfNode ukfnode;
bool init = false;
State state;
sensor_msgs::PointCloud2 pc2, my_map;
geometry_msgs::PoseStamped my_pose, my_pose_est;
tf2::Quaternion myQuaternion;

MyPointCloud2D createScanPoints(sensor_msgs::PointCloud msg)
{
  MyPointCloud2D scan_points;
  int i;
  MyPoint temp_pt;
  for (i = 0; i < msg.points.size(); ++i)
  {
    scan_points.ids.push_back(i);
    scan_points.weights.push_back(1);
    scan_points.distances.push_back(0);
    temp_pt.x = msg.points[i].x;
    temp_pt.y = msg.points[i].y;
    scan_points.pts.push_back(temp_pt);
  }
  scan_points.size = i;

  return scan_points;
}

void readMap()
{
  double x, y;
  MyPoint map_pt;
  io::CSVReader<2> in2("src/scan_matching/map/kreuzung_utm.csv");
  //Lat x, Long y
  in2.read_header(io::ignore_extra_column, "x", "y");
  int i = 0;
  while (in2.read_row(x, y))
  {
    map_carpark.ids.push_back(i);
    vector<double> utm_pt;
    vector<double> utm_local_pt;
    utm_pt.push_back(x); // Lat
    utm_pt.push_back(y); // Long
    utm_pt.push_back(0);
    utm_local_pt = ukfnode.UTM2Local(utm_pt);
    map_pt.x = utm_local_pt[0];
    map_pt.y = utm_local_pt[1];
    map_carpark.pts.push_back(map_pt);
    map_carpark.weights.push_back(1);
    map_carpark.distances.push_back(0);
    i++;
  }
  my_map = rviz.createPointCloud(map_carpark, "ibeo_lux");

  map_carpark.size = i;
}

sensor_msgs::PointCloud filterPC(sensor_msgs::PointCloud pc)
{
  sensor_msgs::PointCloud new_pc;
  new_pc.header = pc.header;
  geometry_msgs::Point32 point;
  sensor_msgs::ChannelFloat32 channel;

  for (int i = 0; i < pc.channels[0].values.size(); i++)
  {
    if (!(pc.channels[0].values[i] == 0 || pc.channels[0].values[i] == 6 || pc.channels[0].values[i] == 4 || pc.channels[0].values[i] == 5 || pc.channels[0].values[i] == 7))
    {
      channel.values.push_back(pc.channels[0].values[i]);
      new_pc.points.push_back(pc.points[i]);
    }
  }
  new_pc.channels.push_back(channel);
  return new_pc;
}

vector<double> setEgoPose(const gpsData::ConstPtr &gps_data)
{
  vector<double> wgs84_pt;
  vector<double> utm_pt;
  vector<double> utm_local_pt;
  //Transform from WGS84 to UTM
  wgs84_pt.push_back(gps_data->ins_pos_abs.In_Long);
  wgs84_pt.push_back(gps_data->ins_pos_abs.In_Lat);
  wgs84_pt.push_back(gps_data->ins_angle_gps_course.Angle_Yaw);
  wgs84_pt.push_back(32);
  //Adjust for Meridian Convergence_
  double utm_yaw = ukfnode.wgs84yaw2utm(wgs84_pt);
  wgs84_pt.pop_back();
  ukfnode.wgs84Toutm(wgs84_pt, utm_pt);

  //Transform from UTM to Local
  utm_local_pt = ukfnode.UTM2Local(utm_pt);
  utm_local_pt[2] = utm_yaw;

  //Set Pose for rviz (debugging)
  my_pose.pose.position.x = utm_local_pt[0];
  my_pose.pose.position.y = utm_local_pt[1];
  my_pose.pose.position.z = 1;
  my_pose.pose.orientation.z = 0;
  my_pose.pose.orientation.y = 1;
  my_pose.pose.orientation.x = 0;
  my_pose.pose.orientation.w = 1;

  return utm_local_pt;
}

float RandomFloat(float a, float b)
{
  float random = ((float)rand()) / (float)RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

void callback(const PointCloud2::ConstPtr &point_cloud, const Marker::ConstPtr &marker, const gpsData::ConstPtr &gps_data)
{
  if (!init)
  {
    readMap();
    init = true;
  }

  //Calc ego position
  vector<double> ego_pos = setEgoPose(gps_data);
  //Set State
  state.x = ego_pos[0];
  state.y = ego_pos[1];
  state.v = gps_data->ins_vh.In_VXH;
  state.yaw = ego_pos[2];
  state.yawr = gps_data->rateshorizontal.RZH;

  //Transform Pointcloud2 into Pointcloud
  sensor_msgs::PointCloud pc;
  sensor_msgs::convertPointCloud2ToPointCloud(*point_cloud, pc);
  //Filter roof scans
  pc = filterPC(pc);

  //Create ScanPoints vector for better handling
  MyPointCloud2D scans = createScanPoints(pc);

  //------Algorithm
  //--1--Reducing ScanPoints
  scans = filter.getScanPointsWithinThreshold(scans);
  //--2--Pre-Allignment (Vorausrichtung)
  Matrix2f rotM = filter.allignScanPoints(scans, state);
  // //--3--Reduce points in map
  MyPointCloud2D map_filt = filter.reduceMap(map_carpark, state);
  my_map = rviz.createPointCloud(map_filt, "ibeo_lux");
  // //--4--ICP Algorithmen
  State new_state;
  scans = icp.mainAlgorithm(map_filt, scans, state, new_state);
  pc2 = rviz.createPointCloud(scans, "ibeo_lux");
  // cout << "Old x: " << state.x << " New x: " << new_state.x << endl;
  // cout << "Old y: " << state.y << " New y: " << new_state.y << endl;

  //Set Pose_est for rviz (debugging)
  my_pose_est.pose.position.x = new_state.x;
  my_pose_est.pose.position.y = new_state.y;
  my_pose_est.pose.position.z = 1;
  my_pose_est.pose.orientation.z = 0;
  my_pose_est.pose.orientation.y = 1;
  my_pose_est.pose.orientation.x = 0;
  my_pose_est.pose.orientation.w = 1;
}

int main(int argc, char **argv)
{
  //Node Handling
  ros::init(argc, argv, "scan_matching_node");
  ros::NodeHandle nh;

  message_filters::Subscriber<PointCloud2> point_cloud_sub(nh, "/as_tx/point_cloud", 1);
  message_filters::Subscriber<Marker> marker_sub(nh, "/as_tx/object_contour_points", 1);
  message_filters::Subscriber<PoseStamped> pose_sub(nh, "vehicle_pose", 1); // Später einführen!
  message_filters::Subscriber<gpsData> gps_sub(nh, "/can_2_ros_gps", 1);

  typedef sync_policies::ApproximateTime<PointCloud2, Marker, gpsData> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), point_cloud_sub, marker_sub, gps_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  auto pc_pub = nh.advertise<sensor_msgs::PointCloud2>("my_point_cloud", 30);
  auto map_pub = nh.advertise<sensor_msgs::PointCloud2>("my_map", 30);
  auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("my_pose", 30);
  auto pose_est_pub = nh.advertise<geometry_msgs::PoseStamped>("my_pose_est", 30);
  ros::Rate rate(30);
  while (ros::ok())
  {
    pc2.header.stamp = ros::Time::now();
    pc_pub.publish(pc2);
    my_map.header.stamp = ros::Time::now();
    map_pub.publish(my_map);
    my_pose.header.stamp = ros::Time::now();
    my_pose.header.frame_id = "ibeo_lux";
    pose_pub.publish(my_pose);
    my_pose_est.header.stamp = ros::Time::now();
    my_pose_est.header.frame_id = "ibeo_lux";
    pose_est_pub.publish(my_pose_est);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <ukf_state_msg/State.h>
#include <ukf2/ukf.h>
#include <geodesy/utm.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <Model_Development_bridge/autobox_out.h>
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
using namespace Model_Development_bridge;

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
ros::Publisher my_true_path;
ros::Publisher my_est_path;
nav_msgs::Path my_est_path_msg, my_true_path_msg;
UKF ukf_filter;

MyPointCloud2D createScanPoints(sensor_msgs::PointCloud msg)
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

void readMapUTM() //-1440, -4200
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
}

void rotatePoint(double &x, double &y, double alpha)
{
  Vector2d temp;
  Vector2d result;
  Matrix2d rot_M;
  alpha = alpha * M_PI / 180;
  rot_M << cos(alpha),
      -sin(alpha),
      sin(alpha), cos(alpha);
  temp << x, y;
  result = rot_M * temp;
  x = result(0);
  y = result(1);
}
void readMapLocal()
{
  double x, y;
  MyPoint map_pt;
  io::CSVReader<2> in2("src/scan_matching/map/parkhaus.csv");
  //Local X , Local Y
  in2.read_header(io::ignore_extra_column, "x", "y");
  int i = 0;
  while (in2.read_row(x, y))
  {
    map_carpark.ids.push_back(i);
    rotatePoint(x, y, -73.7); //+ gegen Uhrzeiger
    map_pt.x = x + 6.8;       // + obenrechts
    map_pt.y = y - 0.9;       // -minus untenrechts
    map_carpark.pts.push_back(map_pt);
    map_carpark.weights.push_back(1);
    map_carpark.distances.push_back(0);
    i++;
  }
  my_map = rviz.createPointCloud(map_carpark, "ibeo_lux");
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
  utm_local_pt[2] = -(utm_yaw - 90) * M_PI / 180.0;

  //Set Pose for rviz (debugging)
  my_pose.pose.position.x = utm_local_pt[0];
  my_pose.pose.position.y = utm_local_pt[1];
  // cout << my_pose.pose.position.x << " | " << my_pose.pose.position.y << endl;
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
State new_state;
double time_us_;

enum MeasureState
{
  Odo,
  Laser,
  Predict
};
MeasureState measure_state;
void callback2(const autobox_out::ConstPtr &can_data)
{
  cout << "Test" << endl;
}
void callback(const PointCloud2::ConstPtr &point_cloud, const Marker::ConstPtr &marker, const gpsData::ConstPtr &gps_data)
{
  readMapUTM();
  // readMapLocal();
  vector<double> ego_pos = setEgoPose(gps_data);
  // cout << ego_pos[0] << " " << ego_pos[1] << endl;
  double tnow = gps_data->header.stamp.toSec();
  double dt = tnow - ukf_filter.time_us_;
  // cout << can_data->ros_imu_odometrie_msg.rate_horizontal_z << endl;
  // cout << gps_data->rateshorizontal.RZH << endl;
  if (int(gps_data->status.Stat_Byte0_GPS_Mode) > 3)
  {
    if (!init)
    {
      // readMapUTM();
      init = true;
      measure_state = MeasureState::Odo;
      ukf_filter.time_us_ = gps_data->header.stamp.toSec();
      new_state.x = ego_pos[0];
      new_state.y = ego_pos[1];
      new_state.yaw = ego_pos[2];
      ukf_filter.x_ << ego_pos[0], ego_pos[1], gps_data->ins_vh.In_VXH, ego_pos[2], -gps_data->rateshorizontal.RZH * M_PI / 180.0;
      return;
    }
    else
    {
      // cout << "pred vor: " << ukf_filter.x_(3) << endl;
      ukf_filter.Prediction(dt);

      // cout << "pred nach: " << ukf_filter.x_(3) << endl;
      // ukf_filter.x_(2) = gps_data->ins_vh.In_VXH;
      // ukf_filter.x_(4) = -gps_data->rateshorizontal.RZH * M_PI / 180.0;
      // ukf_filter.x_(3) = new_state.yaw + dt * ukf_filter.x_(4);
      // ukf_filter.x_(0) = ukf_filter.x_(0) + ukf_filter.x_(2) * dt * cos(ukf_filter.x_(3));
      // ukf_filter.x_(1) = ukf_filter.x_(1) + ukf_filter.x_(2) * dt * sin(ukf_filter.x_(3));
    }

    if (measure_state == MeasureState::Laser)
    {

      //Calc ego position
      // vector<double> ego_pos = setEgoPose(gps_data);
      // ukf_filter.Prediction(dt);
      state.x = ukf_filter.x_(0);
      state.y = ukf_filter.x_(1);
      state.v = ukf_filter.x_(2);

      // cout << "vor: " << ukf_filter.x_(3) << " " << ukf_filter.x_(4) << endl;
      state.yaw = ukf_filter.x_(3);
      state.yawr = ukf_filter.x_(4);

      // state.x = ego_pos[0];
      // state.y = ego_pos[1];
      // state.v = can_data->ros_can_odometrie_msg.velocity_x;
      // state.yawr = gps_data->rateshorizontal.RZH * M_PI / 180.0;
      // if (int(gps_data->status.Stat_Byte0_GPS_Mode) > 3)
      // {
      //   state.x = ego_pos[0];
      //   state.y = ego_pos[1];
      // }
      // else
      // {

      //   // state.x = new_state.x;
      //   // state.y = new_state.y;
      // }

      // cout << "vor: " << state.x << " " << state.y << endl;
      // state.x = new_state.x + 2 * state.v * dt * cos((state.yaw - 90 + 90));
      // state.y = new_state.y - 2 * state.v * dt * sin((state.yaw - 90 + 90));
      // cout << "nach: " << state.x << " " << state.y << endl;
      // cout << ego_pos[0] << " | " << ego_pos[1] << endl;
      //Set State
      // state.x = ego_pos[0];
      // state.y = ego_pos[1];
      timer.start();
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
      Matrix2d rotM = filter.allignScanPoints(scans, state);
      //--3--Reduce points in map
      MyPointCloud2D map_filt = filter.reduceMap(map_carpark, state);
      my_map = rviz.createPointCloud(map_filt, "ibeo_lux");
      // //--4--ICP Algorithmen
      scans = icp.mainAlgorithm(map_filt, scans, state, new_state, rotM);
      // if (new_state.data_flag == 0)
      // {
      //   double dt2 = timer.stop();
      //   state.yaw = state.yaw + dt2 * ukf_filter.x_(4);
      //   ukf_filter.x_(0) = ukf_filter.x_(0) + ukf_filter.x_(2) * dt2 * cos(state.yaw);
      //   ukf_filter.x_(1) = ukf_filter.x_(1) + ukf_filter.x_(2) * dt2 * sin(state.yaw);
      // }
      // else
      // {
      //   ukf_filter.x_(0) = new_state.x;
      //   ukf_filter.x_(1) = new_state.y;
      // }

      pc2 = rviz.createPointCloud(scans, "ibeo_lux");
      vector<double> meas_data;
      meas_data.push_back(new_state.x);
      meas_data.push_back(new_state.y);
      ukf_filter.UpdateMeasurementLidar(meas_data);
      measure_state = MeasureState::Odo;
    }
    else if (measure_state == MeasureState::Odo)
    {

      vector<double> meas_data;

      meas_data.push_back(gps_data->ins_vh.In_VXH);
      meas_data.push_back(gps_data->rateshorizontal.RZH * M_PI / 180.0);
      // meas_data.push_back(can_data->ros_can_odometrie_msg.velocity_x);
      // meas_data.push_back(-(can_data->ros_imu_odometrie_msg.rate_horizontal_z));
      ukf_filter.UpdateMeasurementCAN2(meas_data);
      // cout << "yaw gps: " << ego_pos[2] << " yaw state: " << ukf_filter.x_(3) << endl;
      measure_state = MeasureState::Odo;
    }

    // Set Pose_est for rviz (debugging)
    // my_pose_est.pose.position.x = ego_pos[0];
    // my_pose_est.pose.position.y = ego_pos[1];
    my_pose_est.pose.position.x = ukf_filter.x_(0);
    my_pose_est.pose.position.y = ukf_filter.x_(1);
    // cout << new_state.x << " | " << new_state.y << endl;
    my_pose_est.pose.position.z = 1;
    my_pose_est.pose.orientation.z = 0;
    my_pose_est.pose.orientation.y = 1;
    my_pose_est.pose.orientation.x = 0;
    my_pose_est.pose.orientation.w = 1;
  }
  ukf_filter.time_us_ = tnow;
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
  // message_filters::Subscriber<autobox_out> can_sub(nh, "/data_out", 10);
  typedef sync_policies::ApproximateTime<PointCloud2, Marker, gpsData> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), point_cloud_sub, marker_sub, gps_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::Subscriber can_sub = nh.subscribe("/data_out", 10, callback2);

  auto pc_pub = nh.advertise<sensor_msgs::PointCloud2>("my_point_cloud", 30);
  auto map_pub = nh.advertise<sensor_msgs::PointCloud2>("my_map", 30);
  auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("my_pose", 30);
  auto pose_est_pub = nh.advertise<geometry_msgs::PoseStamped>("my_pose_est", 30);
  ros::Rate rate(60);
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
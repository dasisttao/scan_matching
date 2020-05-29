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
vector<Particle> my_particles;
geometry_msgs::PoseStamped final_state;
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
  my_map = rviz.createPointCloud(map_carpark, "ibeo_lux", 1);
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
  io::CSVReader<2> in2("src/icp_lokalisierung/scan_matching/map/parkhaus.csv");
  //Local X , Local Y
  in2.read_header(io::ignore_extra_column, "x", "y");
  int i = 0;
  while (in2.read_row(x, y))
  {
    map_carpark.ids.push_back(i);

    rotatePoint(x, y, -72.523); //+ gegen Uhrzeiger
    map_pt.x = x ; //+ 8;         // + obenrechts
    map_pt.y = y ; //- 0.6;       // -minus untenrechts
    map_carpark.pts.push_back(map_pt);
    map_carpark.weights.push_back(1);
    map_carpark.distances.push_back(0);
    i++;
  }
  my_map = rviz.createPointCloud(map_carpark, "ibeo_lux", 1.0);
}

sensor_msgs::PointCloud filterPC(sensor_msgs::PointCloud pc)
{
  sensor_msgs::PointCloud new_pc;
  new_pc.header = pc.header;
  geometry_msgs::Point32 point;
  sensor_msgs::ChannelFloat32 channel;

  for (int i = 0; i < pc.channels[0].values.size(); i++)
  {
    if (!(pc.channels[0].values[i] == 0) || pc.channels[0].values[i] == 6 || pc.channels[0].values[i] == 4 || pc.channels[0].values[i] == 5 || pc.channels[0].values[i] == 7)
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
  my_pose.pose.position.z = 1.1;
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
  GPS
};
MeasureState measure_state;

void createParticles(vector<Particle> &particles, const State &state, MyPointCloud2D scans)
{
  particles.clear();
  double offset = 0.6;

  Particle particle(state.x, state.y, state.v, state.yaw, state.yawr, 0, 0, scans);
  particles.push_back(particle);
  // particles.push_back(Particle(state.x, state.y, state.v, state.yaw, state.yawr, 0, offset * 1.5, scans));
  // particles.push_back(Particle(state.x, state.y, state.v, state.yaw, state.yawr, 0, -offset * 1.5, scans));
  // particles.push_back(Particle(state.x, state.y, state.v, state.yaw, state.yawr, 0, offset, scans));
  // particles.push_back(Particle(state.x, state.y, state.v, state.yaw, state.yawr, 0, -offset, scans));
}

void dynamikAdjustUKF(double yawr)
{
  // cout << yawr << endl;
  // double test = 0.3 * abs(yawr) + 5.01;
  // ukf_filter.UpdateLiderParamter(test);
  // std_laspx_ = 0.04;
  // std_laspy_ = std_laspx_;
  // std_lasyaw_ = 0.025;
  // std_odo_yawr = 0.01;
  // std_odo_v = 0.05;
}
bool init_allign = false; 
double time_start = 0;
class Plausible
{
public:
  double x1;
  double x2;
  double y1;
  double y2;
  double yaw1;
  double yaw2;
  double v1;
  double v2;
};
Plausible laser;
Plausible odo;
bool only_odo = false;
bool entered_ramp = false;
bool entered_ramp2 = false;
double speedred = 0.90;
bool first_laser = false;
vector<bool> checkIfRamp()
{

  //Obere Rampe (1. Rampe)
  if (entered_ramp == false)
  {
    if (ukf_filter.x_(0) > 37 && ukf_filter.x_(0) < 39)
    {
      if (ukf_filter.x_(1) > -53.5 && ukf_filter.x_(1) < -48.5)
      {
        first_laser = true;
        entered_ramp = true;
        // ukf_filter.x_(2) -= ukf_filter.x_(2) * speedred;
      }
    }
  }
  else
  {
    // ukf_filter.x_(2) -= ukf_filter.x_(2) * speedred;
    if (ukf_filter.x_(0) > 29 && ukf_filter.x_(0) < 30)
    {
      if (ukf_filter.x_(1) > -56 && ukf_filter.x_(1) < -50)
      {
        entered_ramp = false;
      }
    }
  }
  //Untere Rampe
  if (entered_ramp2 == false)
  {
    if (ukf_filter.x_(0) > 32 && ukf_filter.x_(0) < 34)
    {
      if (ukf_filter.x_(1) > -73 && ukf_filter.x_(1) < -68)
      {
        first_laser = true;
        entered_ramp2 = true;
        // cout << "Rampe2!" << endl;
        // ukf_filter.x_(2) -= ukf_filter.x_(2) * speedred;
      }
    }
  }
  else
  {
    // ukf_filter.x_(2) -= ukf_filter.x_(2) * speedred;
    if (ukf_filter.x_(0) > 42.5 && ukf_filter.x_(0) < 44)
    {
      if (ukf_filter.x_(1) > -70 && ukf_filter.x_(1) < -65.5)
      {
        entered_ramp2 = false;
      }
    }
  }
  // if (entered_ramp || entered_ramp2)
  // {
  //   return true;
  // }
  vector<bool> result;
  result.push_back(entered_ramp);
  result.push_back(entered_ramp2);
  return result;
}

void callback(const PointCloud2::ConstPtr &point_cloud, const gpsData::ConstPtr &gps_data, const autobox_out::ConstPtr &can_data)
{

  vector<double> ego_pos = setEgoPose(gps_data);
  double tnow = gps_data->header.stamp.toSec();
  double dt = tnow - ukf_filter.time_us_;
  if (!init)
  {
    // readMapUTM();
    readMapLocal();
    init = true;
    measure_state = MeasureState::Odo;
    ukf_filter.time_us_ = gps_data->header.stamp.toSec();
    time_start = gps_data->header.stamp.toSec();
    new_state.x = ego_pos[0];
    new_state.y = ego_pos[1];
    new_state.yaw = ego_pos[2];

    ukf_filter.x_ << ego_pos[0], ego_pos[1], gps_data->ins_vh.In_VXH, ego_pos[2], -gps_data->rateshorizontal.RZH * M_PI / 180.0;
    return;
  }

  //FINAL STATE OUTPUT
  final_state.header.seq = 0;
  final_state.header.stamp = ros::Time::now();
  final_state.header.frame_id = "UTM";

  final_state.pose.orientation.w = 0;
  final_state.pose.orientation.x = 0;
  final_state.pose.orientation.y = 0;
  final_state.pose.orientation.z = 0;
  vector<double> temp_pos;

temp_pos.push_back(ukf_filter.x_(0));	
temp_pos.push_back(ukf_filter.x_(1));
temp_pos.push_back(ukf_filter.x_(3));



  vector<double> pos_final = ukfnode.Local2UTM(temp_pos);
  final_state.pose.position.x = pos_final[0];
  final_state.pose.position.y = pos_final[1];
  double winkel_end = (atan2(sin(pos_final[2]), cos(pos_final[2])) * 180.0 / M_PI) + 360 - 72.523;
  if (winkel_end > 360)
  {
    winkel_end -= 360;
  }
  final_state.pose.position.z = winkel_end;

  // dynamikAdjustUKF(ukf_filter.x_(4));
  if (int(gps_data->status.Stat_Byte0_GPS_Mode) < 4)
  {
    first_laser = true;
    vector<bool> ramp = checkIfRamp();
    // ukf_filter.x_(2) -= ukf_filter.x_(2) * 0.25;
    if (measure_state == MeasureState::Laser)
    {
      ukf_filter.Prediction(dt);
      //Für Plausibilität
      laser.x1 = ukf_filter.x_(0);
      laser.y1 = ukf_filter.x_(1);
      laser.yaw1 = ukf_filter.x_(3);
      laser.v1 = ukf_filter.x_(2);
      odo.x1 = laser.x1;
      odo.y1 = laser.y1;
      odo.yaw1 = laser.yaw1;
      odo.v1 = laser.v1;
      //Für Plausibilität ende
      //Setup current state
      state.x = ukf_filter.x_(0);
      state.y = ukf_filter.x_(1);
      state.v = ukf_filter.x_(2);
      state.yaw = ukf_filter.x_(3);
      state.yawr = ukf_filter.x_(4);

      //Transform Pointcloud2 into Pointcloud
      sensor_msgs::PointCloud pc;
      sensor_msgs::convertPointCloud2ToPointCloud(*point_cloud, pc);
      //Filter roof scans
      pc = filterPC(pc);

      //Create ScanPoints vector for better handling
      MyPointCloud2D scans = createScanPoints(pc);
      //------Algorithm
      Particle best_particle;
      MyPointCloud2D map_filt;
      if (!init_allign)
      {
        //--1--Reducing ScanPoints
        filter.lateral_max = 8;
        filter.longi_min = -8;
        filter.longi_max = 8;
        filter.lateral_min = -8;
        filter.map_threshold = 8;
        //--1--Reducing ScanPoints
        scans = filter.getScanPointsWithinThreshold(scans);
        //--2a--Create particles
        createParticles(my_particles, state, scans);
        //--2b--Pre-Allignment (Vorausrichtung)
        Matrix2d rotM = filter.allignScanPoints(scans, state);
        //--2c--Pre-Allign Particles
        filter.allignParticles(my_particles);
        //--3--Reduce points in map
        map_filt = filter.reduceMap(map_carpark, state);
        // my_map = rviz.createPointCloud(map_filt, "ibeo_lux", 1.0);
        // //--4--ICP Algorithmen
        best_particle = icp.particleFilter(map_filt, my_particles);
        //ukf_filter.x_(0) = best_particle.state.x;
        //ukf_filter.x_(1) = best_particle.state.y;
        //ukf_filter.time_us_ = tnow;
        init_allign = true;
        filter.lateral_max = 75;
        filter.longi_min = -75;
        filter.longi_max = 75;
        filter.lateral_min = -75;
        filter.map_threshold = 75;

        //return;
      }

      //--1--Reducing ScanPoints
      scans = filter.getScanPointsWithinThreshold(scans);
      //--2a--Create particles
      createParticles(my_particles, state, scans);
      //--2b--Pre-Allignment (Vorausrichtung)
      Matrix2d rotM = filter.allignScanPoints(scans, state);
      //--2c--Pre-Allign Particles
      filter.allignParticles(my_particles);
      //--3--Reduce points in map
      map_filt = filter.reduceMap(map_carpark, state);
      my_map = rviz.createPointCloud(map_filt, "ibeo_lux", 1.0);
      // //--4--ICP Algorithmen
      best_particle = my_particles[0];

      scans = icp.mainAlgorithm(map_filt, best_particle.pc, best_particle.state, new_state, best_particle.rotM);
      // scans = icp.mainAlgorithm(map_filt, scans, state, new_state, rotM);
      pc2 = rviz.createPointCloud(scans, "ibeo_lux", 1.1);

      vector<double> meas_data;
      meas_data.push_back(new_state.x);
      meas_data.push_back(new_state.y);
      meas_data.push_back(new_state.yaw);
      ukf_filter.UpdateLidar(meas_data);
      ukf_filter.x_(3) = atan2(sin(ukf_filter.x_(3)), cos(ukf_filter.x_(3)));
      // cout << ukf_filter.x_(3) << endl;
      // if (new_state.data_flag == 0)
      // {
      //   double dt2 = timer.stop();
      //   tnow = tnow + dt2;
      //   ukf_filter.Prediction(dt2);
      //   vector<double> meas_data;
      //   meas_data.push_back(can_data->ros_can_odometrie_msg.velocity_x);
      //   meas_data.push_back(-(can_data->ros_can_odometrie_msg.yaw_rate * M_PI / 180.0));
      //   ukf_filter.UpdateMeasurementCAN2(meas_data);
      // }
      // else
      // {
      //   //Für Plausibilität Odo
      //   vector<double> meas_data;
      //   meas_data.push_back(can_data->ros_can_odometrie_msg.velocity_x);
      //   meas_data.push_back(-(can_data->ros_can_odometrie_msg.yaw_rate * M_PI / 180.0));
      //   meas_data.push_back(only_odo);
      //   vector<double> results = ukf_filter.UpdateCANNot(meas_data);
      //   odo.x2 = results[0];
      //   odo.y2 = results[1];
      //   odo.v2 = results[2];
      //   odo.yaw2 = results[3];
      //   //Für Plausibilität Odo ende
      //   if (!only_odo)
      //   {

      //     vector<double> meas_data;
      //     meas_data.push_back(new_state.x);
      //     meas_data.push_back(new_state.y);
      //     meas_data.push_back(new_state.yaw);
      //     ukf_filter.UpdateLidar(meas_data);
      //     ukf_filter.x_(3) = atan2(sin(ukf_filter.x_(3)), cos(ukf_filter.x_(3)));
      //   }
      //   else
      //   {
      //     if (first_laser)
      //     {
      //       first_laser = false;
      //       // vector<double> meas_data;
      //       // meas_data.push_back(new_state.x);
      //       // meas_data.push_back(new_state.y);
      //       // meas_data.push_back(new_state.yaw);
      //       // // ukf_filter.UpdateLidar(meas_data);
      //       // ukf_filter.UpdateLidarAngle(meas_data);
      //       // // ukf_filter.x_(3) = atan2(sin(ukf_filter.x_(3)), cos(ukf_filter.x_(3)));
      //       // // ukf_filter.x_(0) = odo.x2;
      //       // // ukf_filter.x_(1) = odo.y2;
      //       // ukf_filter.x_(2) = odo.v2;

      //       vector<double> meas_data;
      //       meas_data.push_back(can_data->ros_can_odometrie_msg.velocity_x);
      //       meas_data.push_back(-(can_data->ros_can_odometrie_msg.yaw_rate * M_PI / 180.0));
      //       meas_data.push_back(only_odo);
      //       ukf_filter.UpdateCAN(meas_data);
      //     }
      //     else
      //     {
      //       vector<double> meas_data;
      //       meas_data.push_back(can_data->ros_can_odometrie_msg.velocity_x);
      //       meas_data.push_back(-(can_data->ros_can_odometrie_msg.yaw_rate * M_PI / 180.0));
      //       meas_data.push_back(only_odo);
      //       ukf_filter.UpdateCAN(meas_data);
      //     }
      //   }
      //   //Für Plausbilität Laser
      //   laser.x2 = ukf_filter.x_(0);
      //   laser.y2 = ukf_filter.x_(1);
      //   laser.v2 = ukf_filter.x_(2);
      //   laser.yaw2 = ukf_filter.x_(3);
      //   //Für Plausibilität Laser ende
      // }

      measure_state = MeasureState::Odo;
    }
    else if (measure_state == MeasureState::Odo || measure_state == MeasureState::GPS)
    {
      ukf_filter.Prediction(dt);
      vector<double> meas_data;
      double old_yaw = ukf_filter.x_(3);
      meas_data.push_back(can_data->ros_can_odometrie_msg.velocity_x);
      meas_data.push_back(-(can_data->ros_can_odometrie_msg.yaw_rate * M_PI / 180.0));

      ukf_filter.UpdateCAN(meas_data, ramp);
      // ukf_filter.x_(3) = old_yaw;
      measure_state = MeasureState::Laser;
    }
    //Plausiblitäts Check#
    //Delta Laser
    double dxl = laser.x2 - laser.x1;
    double dyl = laser.y2 - laser.y1;
    double dvl = laser.v2 - laser.v1;
    double dyawl = laser.yaw2 - laser.yaw1;
    //Delta Odo
    double dxo = odo.x2 - odo.x1;
    double dyo = odo.y2 - odo.y1;
    double dvo = odo.v2 - odo.v1;
    double dyawo = odo.yaw2 - odo.yaw1;
    //Delta Laser-Odo
    double ddx = abs(dxl) - abs(dxo);
    double ddy = abs(dyl) - abs(dyo);
    double ddv = abs(dvl) - abs(dvo);
    double ddyaw = abs(dyawl) - abs(dyawo);

    vector<double> state;
    state.push_back(ukf_filter.x_(0));
    state.push_back(ukf_filter.x_(1));
    state.push_back(can_data->ros_can_odometrie_msg.velocity_x);
    state.push_back(ukf_filter.x_(3));
    state.push_back(-(can_data->ros_can_odometrie_msg.yaw_rate * M_PI / 180.0));
    bool correctionFlag = false;
    //Delta Laser
    // if (abs(dxl) > 0.1)
    // {
    //   cout << "Laser dxl: " << dxl << endl;
    //   // state[0] += dxl;
    // }
    // if (abs(dyl) > 0.1)
    // {
    //   cout << "Laser dyl: " << dyl << endl;
    //   // state[0] += dyl;
    // }
    // if (abs(dyawl) * 180 / M_PI > 3)
    // {
    //   cout << "Laser dyawl: " << dyawl << endl;
    // }
    // if (abs(dvl) > 0.1)
    // {
    //   cout << "Laser dvl: " << dyl << endl;
    // }

    // if (abs(ddx) > 0.1)
    // {
    //   cout << "ddx: " << ddx << endl;
    //   // state[0] -= ddx;
    //   correctionFlag = true;
    // }

    // if (abs(ddy) > 0.1)
    // {
    //   cout << "ddy: " << ddy << endl;
    //   // state[1] -= ddy;
    //   correctionFlag = true;
    // }
    // //Delta Laser-Odo
    // if (abs(ddv) > 0.1)
    // {
    //   cout << "ddv: " << ddx << endl;
    //   // ukf_filter.x_(2) -= ddv / 2;
    // }

    // if (abs(ddyaw) * 180 / M_PI > 3)
    // {
    //   cout << "ddyaw: " << ddyaw * 180 / M_PI << endl;
    //   // correctionFlag = true;
    // }
    // if (correctionFlag == true)
    // {
    //   ukf_filter.UpdateCorrection(state);
    // }
  }
  else if (int(gps_data->status.Stat_Byte0_GPS_Mode) >= 4)
  {
    ukf_filter.x_ << ego_pos[0], ego_pos[1], gps_data->ins_vh.In_VXH, ego_pos[2], -gps_data->rateshorizontal.RZH * M_PI / 180.0;
  }
  else
  {
    if (measure_state == MeasureState::GPS)
    {
      ukf_filter.Prediction(dt);
      vector<double> meas_data;
      meas_data.push_back(ego_pos[0]);
      meas_data.push_back(ego_pos[1]);
      meas_data.push_back(gps_data->ins_vh.In_VXH);
      meas_data.push_back(ego_pos[2]);
      meas_data.push_back(-(can_data->ros_can_odometrie_msg.yaw_rate * M_PI / 180.0));
      ukf_filter.UpdateGPS(meas_data);
      measure_state = MeasureState::GPS;
    }
    else if (measure_state == MeasureState::Odo)
    {
      ukf_filter.Prediction(dt);
      vector<double> meas_data;
      meas_data.push_back(can_data->ros_can_odometrie_msg.velocity_x);
      meas_data.push_back(-(can_data->ros_can_odometrie_msg.yaw_rate * M_PI / 180.0));
      vector<bool> dummy;
      dummy.push_back(false);
      dummy.push_back(false);
      ukf_filter.UpdateCAN(meas_data, dummy);

      measure_state = MeasureState::GPS;
    }
  }

  //Plausiblitäts Check ende

  //Rampe Prediction
  // double time_passed = gps_data->header.stamp.toSec() - time_start;
  // cout << can_data->ros_can_odometrie_msg.velocity_x << endl;
  // if ((time_passed > 51 && time_passed < 56) || (time_passed > 66 && time_passed < 69) || (time_passed > 83 && time_passed < 88))
  // {
  //   double ds = ukf_filter.x_(2) * dt * ((1 / cos(40 * M_PI / 180.0)) - 1);
  //   ukf_filter.x_(0) += ds * 1000; //cos(ukf_filter.x_(3) + 1);
  //   // ukf_filter.x_(1) += ds * sin(ukf_filter.x_(3));
  // }

  // Set Pose_est for rviz (debugging)
  my_pose_est.pose.position.x = ukf_filter.x_(0);
  my_pose_est.pose.position.y = ukf_filter.x_(1);
  my_pose_est.pose.position.z = 1;
  my_pose_est.pose.orientation.z = 0;
  my_pose_est.pose.orientation.y = 1;
  my_pose_est.pose.orientation.x = 0;
  my_pose_est.pose.orientation.w = 1;

  ukf_filter.time_us_ = tnow;
}

int main(int argc, char **argv)
{
  //Node Handling
  ros::init(argc, argv, "scan_matching_node");
  ros::NodeHandle nh;

  message_filters::Subscriber<PointCloud2> point_cloud_sub(nh, "/as_tx/point_cloud", 10);
  // message_filters::Subscriber<PoseStamped> pose_sub(nh, "vehicle_pose", 10); // Später einführen!
  message_filters::Subscriber<gpsData> gps_sub(nh, "/can_2_ros_gps", 10);
  message_filters::Subscriber<autobox_out> can_sub(nh, "/data_out", 10);
  typedef sync_policies::ApproximateTime<PointCloud2, gpsData, autobox_out> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_cloud_sub, gps_sub, can_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  auto posepubUTM = nh.advertise<geometry_msgs::PoseStamped>("VehiclePoseFusionUTM", 10);
  auto pc_pub = nh.advertise<sensor_msgs::PointCloud2>("my_point_cloud", 30);
  auto map_pub = nh.advertise<sensor_msgs::PointCloud2>("my_map", 30);
  auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("my_pose", 30);
  auto pose_est_pub = nh.advertise<geometry_msgs::PoseStamped>("my_pose_est", 30);
  ros::Rate rate(60);
  while (ros::ok())
  {

    posepubUTM.publish(final_state);
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

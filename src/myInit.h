#ifndef MY_INIT_H
#define MY_INIT_H

#include <iostream>
#include <ros/ros.h>
#include <ukf/ukf.h>
#include <map/map.hpp>


#include <scan_match/utils.hpp>

/**
 * The global value for node
*/
bool initial_pose_needed {true}; // if pose needed to be given by RVIZ 2D Pose Estimation

UKF ukf_filter;
State state, icp_state;
geometry_msgs::PoseStamped gps_pose, pose_estimation, pose_test;
RVIZ rviz;
MyMap my_map;
sensor_msgs::PointCloud2 pc2, map_pc;
MyPointCloud2D map_carpark;  // this is for save offline map, it can change when the floor change. Check Floor.h
bool map_carpark_change {false}; 

class MYINIT{
public:

  MYINIT(const ros::NodeHandle& nh,ros::Rate &rate, const ros::Publisher & map_pub){
    // Read Offline Map to map_pc
    loadOfflineMap();

    getInitWayFromTerminal();

    if(initial_way == InitialWay::RVIZ_GIVE){
       initWithRVIZ(rate, map_pub);
      std::cout << "\n PLZ USE 2D POSE ESTIMATE IN RVIZ TO SET INITAL POSE AND THEN RUN ROSBAG!\n"<<std::endl;
    }
    else if (initial_way == InitialWay::CONFIG_FILE){
      initWithConfigFile(nh);
      std::cout << "\n You CAN RUN ROSBAG DIRECTRLY!\n"<<std::endl;
    }

  }

  enum InitialWay{
    RVIZ_GIVE,
    CONFIG_FILE
  };

  InitialWay initial_way;



  void getInitWayFromTerminal(){
    std::cout << "User must manually set the initial pose before the rosbag play. "<< std::endl;
    std::cout << "\n !!!!! Setting the way you want to initialize.\n Type in r or R : Initialize with RVIZ \n Type in c or C: Initialize with Config File !!!!!!\n"<< std::endl;
    std::string type_in;
    std::cin >> type_in;

    if((type_in == "r") || (type_in == "R")){
      initial_way = InitialWay::RVIZ_GIVE;
      initial_pose_needed = true;
      std::cout << "\n Using the 2D-Estimate in RVIZ to initialize the inital pose of the car!\n"<<std::endl;
    }
    else if((type_in == "c") || (type_in == "C")){
      initial_way = InitialWay::CONFIG_FILE;
      initial_pose_needed = false;
      std::cout << "\n Using the my_init.yaml to initialize the inital pose of the car!\n"<<std::endl;
    }
    else{
      throw std::invalid_argument( "received invalid input for initialization!\n" );
    }

  }



  void initWithConfigFile(const ros::NodeHandle& nh){
    double position_x;
    nh.getParam("position_x",position_x);
    double position_y;
    nh.getParam("position_y",position_y);
    double v;
    nh.getParam("v",v);
    double yaw;
    nh.getParam("yaw",yaw);
    double yawr;
    nh.getParam("yawr",yawr);

    std::cout <<  " Initialized X:   "<< position_x << std::endl;
    std::cout <<  " Initialized Y:   "<< position_y << std::endl;
    std::cout <<  " Initialized V: "<< v << std::endl;
    std::cout <<  " Initialized Yaw: "<< yaw << std::endl;
    std::cout <<  " Initialized Yawr: "<< yawr << std::endl;

    ukf_filter.x_ << position_x, position_y, v, yaw, yawr;
    ukf_filter.P_ << 10, 0.03, -0.014, 0, 0,
        0.03, 10, -0.01, 0, 0,
        -0.014, 0, 0.05, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0;

    state.x = ukf_filter.x_(0);
    state.y = ukf_filter.x_(1);
    state.v = ukf_filter.x_(2);
    state.yaw = ukf_filter.x_(3);
    state.yawr = ukf_filter.x_(4);

    //Display estimated pose on rviz (here it is inital pose)
    rviz.displayEstimatedPose(state, pose_estimation);
  }



  void initWithRVIZ(ros::Rate &rate, const ros::Publisher & map_pub){
    while(ros::ok()){
      // Wait till callback_init_pose is called.
      if(initial_pose_needed == false){break;}

      map_pc.header.stamp = ros::Time::now();
      map_pub.publish(map_pc);

      ros::spinOnce();
      rate.sleep();
    }
  }

  void loadOfflineMap(){
    std::cout << "Loading Offline Map ..."<< std::endl;
    my_map.readRawData(map_pc, map_carpark,"src/icp_lokalisierung/scan_matching/map/map_Elphi_R0_E1_Bauplan_reduced.csv");
    map_carpark_change = true;
    std::cout << "  Map loaded!"<< std::endl;
  }


};



#endif
#include <ukf/ukf.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros_can_gps_msg/gpsData.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ukf_state_msg/State.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <Model_Development_bridge/autobox_out.h>

using namespace std;
using namespace ros_can_gps_msg;
using namespace ukf_state_msg;
using namespace message_filters;
using namespace Eigen;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace Model_Development_bridge;

ros::Publisher my_pose_pub;
ros::Publisher my_path_pub;
ros::Publisher my_ukf_state_pub;
ros::Publisher my_true_path_pub;
nav_msgs::Path my_path, my_true_path;

PoseStamped my_pose;
State my_ukf_state;
vector<double> local_gps_pos;
#include <ukf2/ukf.h>
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

// void publish_state()
// {
// 	if (!ukf_filter.is_initialized_)
// 		return;
// 	else
// 	{

// 		//State rviz
// 		my_pose.header.seq = 0;
// 		ros::Time time_stamp = ros::Time::now();
// 		my_pose.header.stamp = time_stamp;
// 		my_pose.header.frame_id = "ibeo_lux";
// 		my_pose.pose.position.x = ukf_filter.x_(0);
// 		my_pose.pose.position.y = ukf_filter.x_(1);
// 		my_pose.pose.position.z = 1;
// 		my_pose.pose.orientation.x = 0;
// 		my_pose.pose.orientation.y = 1;
// 		my_pose.pose.orientation.z = 0;
// 		my_pose.pose.orientation.w = 1;

// 		// //UKF State
// 		// my_ukf_state.header = my_pose.header;
// 		// my_ukf_state.x = ukf_filter.x_(0);
// 		// my_ukf_state.y = ukf_filter.x_(1);
// 		// my_ukf_state.v = ukf_filter.x_(2);
// 		// my_ukf_state.yaw = ukf_filter.x_(3);
// 		// my_ukf_state.yawr = ukf_filter.x_(4);

// 		//Path UKF
// 		my_path.header.stamp = time_stamp;
// 		my_path.header.frame_id = "ibeo_lux";
// 		geometry_msgs::PoseStamped this_pose_stamped;
// 		this_pose_stamped.pose.position.x = my_pose.pose.position.x;
// 		this_pose_stamped.pose.position.y = my_pose.pose.position.y;
// 		this_pose_stamped.pose.orientation.x = 0;
// 		this_pose_stamped.pose.orientation.y = 1;
// 		this_pose_stamped.pose.orientation.z = 0;
// 		this_pose_stamped.pose.orientation.w = 1;

// 		this_pose_stamped.header.stamp = time_stamp;
// 		this_pose_stamped.header.frame_id = "ibeo_lux";
// 		my_path.poses.push_back(this_pose_stamped);

// 		//True Path
// 		my_true_path.header = my_path.header;

// 		this_pose_stamped.pose.position.x = local_gps_pos[0];
// 		this_pose_stamped.pose.position.y = local_gps_pos[1];
// 		this_pose_stamped.pose.orientation.x = 0;
// 		this_pose_stamped.pose.orientation.y = 1;
// 		this_pose_stamped.pose.orientation.z = 0;
// 		this_pose_stamped.pose.orientation.w = 1;

// 		this_pose_stamped.header.stamp = time_stamp;
// 		this_pose_stamped.header.frame_id = "ibeo_lux";
// 		my_true_path.poses.push_back(this_pose_stamped);

// 		//Publish
// 		my_pose_pub.publish(my_pose);
// 		my_path_pub.publish(my_path);
// 		my_true_path_pub.publish(my_true_path);
// 		// my_ukf_state_pub.publish(my_ukf_state);
// 	}
// }

// enum MeasureState
// {
// 	Odo,
// 	Laser
// };
// MeasureState measure_state;

// void callback(const gpsData::ConstPtr &gps_data, const autobox_out::ConstPtr &can_data)
// {

// 	// cout << "GPS Velo: " << gps_data->ins_vh << endl;
// 	local_gps_pos = getLocalPose(gps_data);
// 	// cout << can_data-> << endl;

// 	cout << can_data->ros_imu_odometrie_msg.vel_horizontal_x << endl;
// 	cout << "Speed GPS: " << local_gps_pos[2] << " , CAN: " << can_data->ros_can_odometrie_msg.velocity_x << endl;
// 	// cout << "Yaw GPS: " << local_gps_pos[3] << " , CAN: " << can_data->ros_can_odometrie_msg.steering_wheel_angle + 90 << endl;
// 	// cout << "Yawrate GPS: " << local_gps_pos[4] << " , CAN: " << can_data->ros_can_odometrie_msg.yaw_rate << endl;
// 	local_gps_pos[2] = can_data->ros_can_odometrie_msg.velocity_x;
// 	local_gps_pos[4] = -can_data->ros_imu_odometrie_msg.rate_horizontal_z;
// 	local_gps_pos[3] = -local_gps_pos[3];
// 	if (!ukf_filter.is_initialized_)
// 	{
// 		measure_state = MeasureState::Odo;
// 		ukf_filter.is_initialized_ = true;
// 		ukf_filter.time_us_ = can_data->header.stamp.toSec();
// 		ukf_filter.x_ << local_gps_pos[0], local_gps_pos[1], local_gps_pos[2], local_gps_pos[3] * M_PI / 180.0, local_gps_pos[4] * M_PI / 180.0;

// 		return;
// 	}
// 	else
// 	{
// 		double tnow = can_data->header.stamp.toSec();
// 		double dt = tnow - ukf_filter.time_us_;

// 		if (dt > 0)
// 		{
// 			ukf_filter.Prediction(dt);
// 			if (measure_state == MeasureState::Odo)
// 			{
// 				cout << "vor: " << ukf_filter.x_(2) << endl;
// 				ukf_filter.UpdateMeasurementCAN2(local_gps_pos);
// 				cout << "nach: " << ukf_filter.x_(2) << endl;
// 				measure_state = MeasureState::Odo; // Später ändern zu Laser!
// 			}
// 			else if (measure_state == MeasureState::Laser)
// 			{
// 				//ukf_filter.UpdateLidar(local_gps_pos);
// 				measure_state = MeasureState::Odo;
// 			}
// 			ukf_filter.time_us_ = tnow;
// 		}
// 	}
// 	// cout << "Measure State: " << measure_state << endl;
// 	// cout << "X GPS: " << local_gps_pos[0] << endl;
// 	// cout << "X State: " << ukf_filter.x_(0) << endl;
// 	// cout << "Y GPS: " << local_gps_pos[1] << endl;
// 	// cout << "Y State: " << ukf_filter.x_(1) << endl;

// 	// cout << "______________" << endl;
// 	publish_state();
// }

// int main(int argc, char **argv)
// {

// 	ros::init(argc, argv, "UKFNode");
// 	ros::NodeHandle nh;

// 	//Subscribers
// 	// ros::Subscriber ukf_sub = nh.subscribe("/can_2_ros_gps", 10, callback);

// 	// message_filters::Subscriber<PointCloud2> point_cloud_sub(nh, "/as_tx/point_cloud", 10);
// 	message_filters::Subscriber<autobox_out> pose_sub(nh, "/data_out", 10); // LASER POSE
// 	message_filters::Subscriber<gpsData> gps_sub(nh, "/can_2_ros_gps", 10); // ODOMETRIE

// 	//Time Sync Subscribers
// 	typedef sync_policies::ApproximateTime<gpsData, autobox_out> MySyncPolicy;
// 	Synchronizer<MySyncPolicy> sync(MySyncPolicy(60), gps_sub, pose_sub);
// 	sync.registerCallback(boost::bind(&callback, _1, _2));

// 	//Publishers
// 	my_pose_pub = nh.advertise<PoseStamped>("my_pose_ukf", 10);
// 	my_path_pub = nh.advertise<nav_msgs::Path>("my_trajectory_ukf", 10, true);
// 	my_ukf_state_pub = nh.advertise<State>("my_ukf_state", 10);
// 	my_true_path_pub = nh.advertise<nav_msgs::Path>("my_true_trajectory_ukf", 10, true);

// 	ros::Rate rate(60);
// 	while (ros::ok())
// 	{

// 		ros::spinOnce();
// 		rate.sleep();
// 	}

// 	return 0;
// }
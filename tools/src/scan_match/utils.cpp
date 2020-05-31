#include <scan_match/utils.hpp>

void Timer::start()
{
    beg = std::chrono::high_resolution_clock::now();
}
double Timer::stop()
{
    end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - beg).count();
    return double(duration) / 1000;
}

sensor_msgs::PointCloud2 RVIZ::createPointCloud(MyPointCloud2D my_pc, string frame_id, float height)
{
    sensor_msgs::PointCloud pc;
    pc.points.resize(my_pc.pts.size());
    pc.header.frame_id = frame_id;
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

MyPointCloud2D ScanPoints::create(sensor_msgs::PointCloud msg)
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

vector<bool> Ramp::check(const State state)
{

    //Obere Rampe (1. Rampe)
    if (entered_ramp == false)
    {
        if (state.x > 37 && state.x < 39)
        {
            if (state.y > -53.5 && state.y < -48.5)
            {
                entered_ramp = true;
            }
        }
    }
    else
    {
        if (state.x > 29 && state.x < 30)
        {
            if (state.y > -56 && state.y < -50)
            {
                entered_ramp = false;
            }
        }
    }
    //Untere Rampe
    if (entered_ramp2 == false)
    {
        if (state.x > 32 && state.x < 34)
        {
            if (state.y > -73 && state.y < -68)
            {
                entered_ramp2 = true;
            }
        }
    }
    else
    {
        if (state.x > 42.5 && state.x < 44)
        {
            if (state.y > -70 && state.y < -65.5)
            {
                entered_ramp2 = false;
            }
        }
    }
    vector<bool> result;
    result.push_back(entered_ramp);
    result.push_back(entered_ramp2);
    return result;
}

void RVIZ::outputLocalizationResult(const State state, geometry_msgs::PoseStamped &result_output)
{
    //Prepare final node output
    result_output.header.seq = 0;
    result_output.header.stamp = ros::Time::now();
    result_output.header.frame_id = "UTM";
    result_output.pose.orientation.w = 0;
    result_output.pose.orientation.x = 0;
    result_output.pose.orientation.y = 0;
    result_output.pose.orientation.z = 0;
    vector<double> temp_pos;
    temp_pos.push_back(state.x);
    temp_pos.push_back(state.y);
    temp_pos.push_back(state.yaw);
    vector<double> pos_final = coord_transform.Local2UTM(temp_pos);
    result_output.pose.position.x = pos_final[0];
    result_output.pose.position.y = pos_final[1];
    double winkel_end = (atan2(sin(pos_final[2]), cos(pos_final[2])) * 180.0 / M_PI) + 360 - 72.523;

    //Hier Evtl richtig normieren! oder auch überprüfen was bei <0 passiert!
    if (winkel_end > 360)
    {
        winkel_end -= 360;
    }
    result_output.pose.position.z = winkel_end;
}
void RVIZ::displayEstimatedPose(const State state, geometry_msgs::PoseStamped &pose_estimation)
{
    pose_estimation.pose.position.x = state.x;
    pose_estimation.pose.position.y = state.y;
    pose_estimation.pose.position.z = 1;
    pose_estimation.pose.orientation.z = 0;
    pose_estimation.pose.orientation.y = 1;
    pose_estimation.pose.orientation.x = 0;
    pose_estimation.pose.orientation.w = 1;
}
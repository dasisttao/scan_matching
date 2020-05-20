#pragma once
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

sensor_msgs::PointCloud2 RVIZ::createPointCloud(MyPointCloud2D my_pc, string frame_id)
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
        pc.points[i].z = 1.1;
        depth_channel.values.push_back(1);
    }
    pc.channels.push_back(depth_channel);
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
    return pc2;
}
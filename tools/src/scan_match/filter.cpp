#include "filter.hpp"
#include <chrono>

RVIZ rviz;

MyPointCloud2D Filter::getScanPointsWithinThreshold(MyPointCloud2D scans)
{
    MyPointCloud2D scans_filt;
    MyPoint temp_pt;
    for (int i = 0; i < scans.pts.size(); i++)
    {
        if (scans.pts[i].x < longi_max && scans.pts[i].x > longi_min)
        {
            if (scans.pts[i].y < lateral_max && scans.pts[i].y > lateral_min)
            {
                scans_filt.distances.push_back(scans.distances[i]);
                scans_filt.ids.push_back(scans.ids[i]);
                temp_pt.x = scans.pts[i].x;
                temp_pt.y = scans.pts[i].y;
                scans_filt.pts.push_back(temp_pt);
                scans_filt.weights.push_back(scans.weights[i]);
            }
        }
    }
    return scans_filt;
}

MyPointCloud2D Filter::getScanPointsWithinThreshold_param(MyPointCloud2D scans, float longiMax, float longiMin, float lateralMax, float lateralMIN)
{
    MyPointCloud2D scans_filt;
    MyPoint temp_pt;
    for (int i = 0; i < scans.pts.size(); i++)
    {
        if (scans.pts[i].x < longiMax && scans.pts[i].x > longiMin)
        {
            if (scans.pts[i].y < lateralMax && scans.pts[i].y > lateralMIN)
            {
                scans_filt.distances.push_back(scans.distances[i]);
                scans_filt.ids.push_back(scans.ids[i]);
                temp_pt.x = scans.pts[i].x;
                temp_pt.y = scans.pts[i].y;
                scans_filt.pts.push_back(temp_pt);
                scans_filt.weights.push_back(scans.weights[i]);
            }
        }
    }
    return scans_filt;
}


// filter 2 ares, which has the same x value area.

MyPointCloud2D Filter::getScanPointsWithinThreshold_param(MyPointCloud2D scans, float longiMax, float longiMin, float lateralMax1, float lateralMIN1, float lateralMax2, float lateralMIN2)
{
    MyPointCloud2D scans_filt;
    MyPoint temp_pt;
    for (int i = 0; i < scans.pts.size(); i++)
    {
        if (scans.pts[i].x < longiMax && scans.pts[i].x > longiMin)
        {
            if( (scans.pts[i].y < lateralMax1 && scans.pts[i].y > lateralMIN1) || ((scans.pts[i].y < lateralMax2 && scans.pts[i].y > lateralMIN2)) )
            {
                scans_filt.distances.push_back(scans.distances[i]);
                scans_filt.ids.push_back(scans.ids[i]);
                temp_pt.x = scans.pts[i].x;
                temp_pt.y = scans.pts[i].y;
                scans_filt.pts.push_back(temp_pt);
                scans_filt.weights.push_back(scans.weights[i]);
            }
        }
    }
    return scans_filt;
}




Matrix2d Filter::allignScanPoints(MyPointCloud2D &scans, const State &state)
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
    for (int i = 0; i < scans.pts.size(); i++)
    {
        temp << scans.pts[i].x, scans.pts[i].y;
        result = rot_M * temp + ego_pos;
        scans.pts[i].x = result(0);
        scans.pts[i].y = result(1);
    }
    return rot_M;
}

MyPointCloud2D Filter::reduceMap(MyPointCloud2D map_carpark, const State &state)
{
    MyPointCloud2D map_filt;
    MyPoint temp_pt;
    for (int i = 0; i < map_carpark.pts.size(); i++)
    {
        if (map_carpark.pts[i].x < state.x + map_threshold &&
            map_carpark.pts[i].x > state.x - map_threshold &&
            map_carpark.pts[i].y < state.y + map_threshold &&
            map_carpark.pts[i].y > state.y - map_threshold)
        {
            map_filt.distances.push_back(map_carpark.distances[i]);
            map_filt.ids.push_back(map_carpark.ids[i]);
            temp_pt.x = map_carpark.pts[i].x;
            temp_pt.y = map_carpark.pts[i].y;
            map_filt.pts.push_back(temp_pt);
            map_filt.weights.push_back(map_carpark.weights[i]);
        }
    }
    return map_filt;
}

sensor_msgs::PointCloud Filter::filterLaserChannel(sensor_msgs::PointCloud pc)
{
    sensor_msgs::PointCloud new_pc;
    new_pc.header = pc.header;
    geometry_msgs::Point32 point;
    sensor_msgs::ChannelFloat32 channel;

    for (int i = 0; i < pc.channels[0].values.size(); i++)
    {
        // IBEO-LUX-8L use two configuration of sensor to get 8 layer pointcloud
        // For Data consistency, the frames which contain layer from 4-7 is ignored
        if (  (pc.channels[0].values[i] == 4) ||
              (pc.channels[0].values[i] == 5) ||
              (pc.channels[0].values[i] == 6) ||
              (pc.channels[0].values[i] == 7) 
        ){
            is_this_frame_has_4to7Layer_PCL = true;
            return new_pc; 
        }
        if (! (
                    (pc.channels[0].values[i] == 0) 
                ||  (pc.channels[0].values[i] == 3) 
              ) 
            )
        {
            channel.values.push_back(pc.channels[0].values[i]);
            new_pc.points.push_back(pc.points[i]);
        }
    }
    is_this_frame_has_4to7Layer_PCL = false;
    new_pc.channels.push_back(channel);
    return new_pc;
}
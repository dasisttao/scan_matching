#include "filter.hpp"
#include <chrono>

RVIZ rviz;
MyPointCloud2D Filter::getScanPointsWithinThreshold(MyPointCloud2D scans)
{
    MyPointCloud2D scans_filt;
    MyPoint temp_pt;
    for (int i = 0; i < scans.size; i++)
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
    scans_filt.size = scans_filt.ids.size();
    return scans_filt;
}

Matrix2f Filter::allignScanPoints(MyPointCloud2D &scans, const State &state)
{
    Matrix2f rot_M;
    Vector2f ego_pos;
    Vector2f temp;
    Vector2f result;
    ego_pos << state.x, state.y;

    double alpha = ((state.yaw - 90) * PI / 180.0);
    rot_M
        << cos(alpha),
        sin(alpha),
        -sin(alpha), cos(alpha);
    for (int i = 0; i < scans.size; i++)
    {
        temp << scans.pts[i].x - 1.9, scans.pts[i].y + 0.4;
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
    for (int i = 0; i < map_carpark.size; i++)
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
    map_filt.size = map_filt.ids.size();
    return map_filt;
}

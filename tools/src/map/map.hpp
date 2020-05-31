#pragma once
#include <ros/ros.h>
#include <csv/csv.h>
#include <scan_match/var_defs.hpp>
#include <scan_match/transform.hpp>
#include <scan_match/utils.hpp>

class MyMap
{
public:
    void readMapCrossroad(sensor_msgs::PointCloud2 &my_map, MyPointCloud2D &map_crossroad);
    void rotatePoint(double &x, double &y, double alpha);
    void readMapParkhaus(sensor_msgs::PointCloud2 &my_map, MyPointCloud2D &map_carpark);

private:
    CoordTransform coord_transform;
    RVIZ rviz;
};

void MyMap::rotatePoint(double &x, double &y, double alpha)
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
void MyMap::readMapCrossroad(sensor_msgs::PointCloud2 &my_map, MyPointCloud2D &map_crossroad) //-1440, -4200
{
    double x, y;
    MyPoint map_pt;
    io::CSVReader<2> in2("src/scan_matching/map/kreuzung_utm.csv");
    //Lat x, Long y
    in2.read_header(io::ignore_extra_column, "x", "y");
    int i = 0;
    while (in2.read_row(x, y))
    {
        map_crossroad.ids.push_back(i);
        vector<double> utm_pt;
        vector<double> utm_local_pt;
        utm_pt.push_back(x); // Lat
        utm_pt.push_back(y); // Long
        utm_pt.push_back(0);
        utm_local_pt = coord_transform.UTM2Local(utm_pt);
        map_pt.x = utm_local_pt[0];
        map_pt.y = utm_local_pt[1];
        map_crossroad.pts.push_back(map_pt);
        map_crossroad.weights.push_back(1);
        map_crossroad.distances.push_back(0);
        i++;
    }
    my_map = rviz.createPointCloud(map_crossroad, "ibeo_lux", 1);
}

void MyMap::readMapParkhaus(sensor_msgs::PointCloud2 &map_pc, MyPointCloud2D &map_carpark)
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
        map_pt.x = x;
        map_pt.y = y;
        map_carpark.pts.push_back(map_pt);
        map_carpark.weights.push_back(1);
        map_carpark.distances.push_back(0);
        i++;
    }
    map_pc = rviz.createPointCloud(map_carpark, "ibeo_lux", 1.0);
}

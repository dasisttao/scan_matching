#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <kdtree/nanoflann.hpp>
#include <chrono>
#include <ukf_state_msg/State.h>
#define PI 3.14159265

using namespace std;
using namespace Eigen;
using namespace nanoflann;
using namespace ukf_state_msg;
template <typename T>
struct PointCloud2D
{
    struct Point
    {
        T x, y;
    };

    std::vector<Point> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return pts[idx].x;
        else
            return pts[idx].y;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloud2D<float>>,
    PointCloud2D<float>, 2 /* dim */
    >
    my_kd_tree_t;

struct MyPoint
{
    double x, y;
};

class MyPointCloud2D
{
public:
    std::vector<int> ids;
    std::vector<float> weights;
    std::vector<float> distances;
    std::vector<MyPoint> pts;
};

class Particle
{
public:
    Particle() = default;
    Particle(double x, double y, double v, double yaw, double yawr, double dx, double dy, MyPointCloud2D pc)
    {
        double alpha = (-yaw);
        Vector2d new_pos;
        Vector2d old_pos;
        Vector2d offset;
        old_pos << x, y;
        offset << dx, dy;
        this->rotM
            << cos(alpha),
            sin(alpha),
            -sin(alpha), cos(alpha);

        new_pos = rotM * offset + old_pos;
        this->state.x = new_pos(0);
        this->state.y = new_pos(1);
        this->state.v = v;
        this->state.yaw = yaw;
        this->state.yawr = yawr;
        this->pc = pc;
    }

public:
    State state;
    float error;
    Matrix2d rotM;
    MyPointCloud2D pc;
};

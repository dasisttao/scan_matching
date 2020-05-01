#pragma once
#include <iostream>
#include <scan_match/var_defs.hpp>
#include <ukf_state_msg/State.h>
#include <scan_match/utils.hpp>

using namespace std;
using namespace ukf_state_msg;

class ICP
{
public:
    ICP() = default;
    void calcEqPoints(MyPointCloud2D map_corrs, MyPointCloud2D scans, Matrix2f &R, Vector2f &T);
    MyPointCloud2D verwerfung(float filt_distance, MyPointCloud2D &map_corrs, const MyPointCloud2D &scans);
    float getFiltDistance(float error_before_matching, float ratio_corres_last_timestep);
    void createPointCloud2D(PointCloud2D<float> &cloudMap, PointCloud2D<float> &cloudScan, const MyPointCloud2D &map_carpark, const MyPointCloud2D &scans);
    MyPointCloud2D findNeigherstNeighbor(const PointCloud2D<float> &map_carpark, const PointCloud2D<float> &scans, float &distance_total_corrs_sqr, const my_kd_tree_t &index);
    void mainAlgorithm(const MyPointCloud2D &map_carpark, const MyPointCloud2D &scans);

private:
    const size_t number_of_iterations = 50;
    const size_t number_of_results = 1;

private:
    Timer timer;
};
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
    void calcEqPoints(MyPointCloud2D map_corrs, MyPointCloud2D scans, Matrix2d &R, Vector2d &T);
    MyPointCloud2D verwerfung(float filt_distance, MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark);
    float getFiltDistance(float error_before_matching, float ratio_corres_last_timestep);
    void createPointCloud2D(PointCloud2D<float> &cloudMap, PointCloud2D<float> &cloudScan, const MyPointCloud2D &map_carpark, const MyPointCloud2D &scans);
    MyPointCloud2D findNeigherstNeighbor(const PointCloud2D<float> &map_carpark, const PointCloud2D<float> &scans, MyPointCloud2D &my_scans, float &distance_total_corrs_sqr, const my_kd_tree_t &index);
    State matchingResult(const vector<Matrix2d> &TR, const vector<Vector2d> &TT, State state);
    void calcWeights(MyPointCloud2D &scans);
    MyPointCloud2D mainAlgorithm(const MyPointCloud2D &map_carpark, MyPointCloud2D &scans, State state, State &new_state);

private:
    const size_t number_of_iterations = 3;
    const size_t number_of_results = 1;

private:
    Timer timer;
};
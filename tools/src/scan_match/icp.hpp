#pragma once
#include <iostream>
#include <scan_match/var_defs.hpp>
#include <ukf_state_msg/State.h>
#include <scan_match/utils.hpp>
#include <scan_match/icp_methods.hpp>

using namespace std;
using namespace ukf_state_msg;

class ICP
{
public:
    ICP() = default;
    void calcEqPoints(MyPointCloud2D map_corrs, MyPointCloud2D scans, Matrix2d &R, Vector2d &T);
    void createPointCloud2D(PointCloud2D<float> &pc, const MyPointCloud2D &my_pc);
    MyPointCloud2D findNeigherstNeighbor(const PointCloud2D<float> &map_carpark, const PointCloud2D<float> &scans, MyPointCloud2D &my_scans, float &distance_total_corrs_sqr, const my_kd_tree_t &index);
    State matchingResult(const vector<Matrix2d> &TR, const vector<Vector2d> &TT, State state, Matrix2d &rotM);
    void calcWeights(MyPointCloud2D &scans);
    MyPointCloud2D mainAlgorithm(const MyPointCloud2D &map_carpark, MyPointCloud2D &scans, State state, State &new_state, Matrix2d &rotM);

private:
    const size_t number_of_iterations = 5;
    const size_t number_of_results = 1;
    const VerwerfungsMethode verwerfungs_methode = VerwerfungsMethode::constantValue;
    const GewichtungsMethode gewichtungs_methode = GewichtungsMethode::applyNothing;

private:
    Timer timer;
    Verwerfung verwerfung;
    Gewichtung gewichtung;
};
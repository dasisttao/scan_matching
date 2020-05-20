#pragma once
#include <iostream>
#include <scan_match/var_defs.hpp>
#include <ukf_state_msg/State.h>
#include <scan_match/utils.hpp>

using namespace std;
using namespace ukf_state_msg;

class Filter
{

public:
    Filter() = default;
    MyPointCloud2D getScanPointsWithinThreshold(MyPointCloud2D scans);
    Matrix2d allignScanPoints(MyPointCloud2D &scans, const State &state);
    void allignParticles(vector<Particle> &particles);
    MyPointCloud2D reduceMap(MyPointCloud2D map_carpark, const State &state);
    void createParticles(vector<Particle> &particles, const State &state);

public:
    float longi_max = 75;
    float longi_min = -75;
    float lateral_max = 75;
    float lateral_min = -75;
    float map_threshold = 75;
};

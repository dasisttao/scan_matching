#pragma once
#include <scan_match/var_defs.hpp>
#include <ukf_state_msg/State.h>
#include <scan_match/utils.hpp>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <random>
#include <string>
#include <vector>

using namespace std;

struct Particle
{
    int id;
    double x;
    double y;
    double theta;
    double weight;
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
};
struct LandmarkObs
{

    int id;   // Id of matching landmark in the map.
    double x; // Local (vehicle coords) x position of landmark observation [m]
    double y; // Local (vehicle coords) y position of landmark observation [m]
};
class ParticleFilter
{
public:
    ParticleFilter() = default;
    void initParticles(double x, double y, double theta);
    void prediction(double delta_t, double velocity, double yaw_rate);
    void updateWeights(double sensor_range, const vector<LandmarkObs> &observations, const MyPointCloud2D &map);
    void resample();
    vector<LandmarkObs> TransformToMapCoords(Particle particle, vector<LandmarkObs> observation);
    // Set of current particles
    std::vector<Particle> particles;

private:
    const double sigma_pos[3] = {0.3, 0.3, 0.01};
    // Landmark measurement uncertainty [x [m], y [m]]
    const double sigma_landmark[2] = {0.3, 0.3};
    // Number of particles to draw
    int num_particles;

    // Flag, if filter is initialized
    bool is_initialized;

    // Vector of weights of all particles
    std::vector<double> weights;
};

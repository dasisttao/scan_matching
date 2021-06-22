#pragma once
#include <iostream>
#include <scan_match/var_defs.hpp>
#include <ukf_state_msg/State.h>
#include <scan_match/utils.hpp>
#include <Model_Development_bridge/autobox_out.h>
#include <sensor_msgs/PointCloud.h>

float Median(const std::vector<float>& v);
float MAD(const std::vector<float>& v);

enum VerwerfungsMethode
{
    constantValue,
    constantAndRisingDistance,
    sigmaFilter
};

enum GewichtungsMethode
{
    applyNothing,
    divideByAllCorrespondingDistances,
    setZeroIfTwoCorrespondings
};
class Verwerfung
{
public:
    MyPointCloud2D selectVewerfung(VerwerfungsMethode verwerfungs_methode, MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark, const State &state);

private:
    MyPointCloud2D constantValue(MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark);
    MyPointCloud2D constantAndRisingDistance(MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark, const State &state);
    MyPointCloud2D sigmaFilter(MyPointCloud2D &map_corrs, const MyPointCloud2D &scans, const MyPointCloud2D &map_carpark);
private:
    float getFiltDistance(float error_before_matching, float ratio_corres_last_timestep);

private:
    //constantValue
    const float constant_value = 0.2;
    //constantAndRisingDistance
    const float min_distance = 0.5;
    const float max_distance = 2;
};

class Gewichtung
{
public:
    void selectGewichtung(GewichtungsMethode gewichtungs_methode, MyPointCloud2D &scans);

private:
    void divideByAllCorrespondingDistances(MyPointCloud2D &scans);
    void setZeroIfTwoCorrespondings(MyPointCloud2D &scans);
    void applyNothing(MyPointCloud2D &scans);
};

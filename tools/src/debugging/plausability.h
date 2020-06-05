#include <ros/ros.h>
#include <iostream>
#include <ukf_state_msg/State.h>
#include <ukf/ukf.h>

using namespace ukf_state_msg;

class Plausability
{
public:
    void setState(const State &state);
    void setStateICP(const UKF &ukf_filter);
    void setStateOdo(const State &state);
    void deltaOdoICP();

public:
    vector<double> times;

private:
    //State at t0
    State state_start;
    //ICP states t1 and t1-t0
    State state_icp;
    State delta_icp;
    //Odo states t1 and t1-t0
    State state_odo;
    State delta_odo;
    //Delta odo_t1 - icp_t1
    State delta_odo_icp;

    vector<State> deltas_icp;
    vector<State> deltas_odo;
    vector<State> deltas_odo_icp;
};

void Plausability::setState(const State &state)
{
    state_start = state;
}

void Plausability::setStateICP(const UKF &ukf_filter)
{
    state_icp.x = ukf_filter.x_(0);
    state_icp.y = ukf_filter.x_(1);
    state_icp.v = ukf_filter.x_(2);
    state_icp.yaw = ukf_filter.x_(3);
    state_icp.yawr = ukf_filter.x_(4);
    //Delta State Laser t1-t0
    delta_icp.x = state_icp.x - state_start.x;
    delta_icp.y = state_icp.y - state_start.y;
    delta_icp.v = state_icp.v - state_start.v;
    delta_icp.yaw = state_icp.yaw - state_start.yaw;
    delta_icp.yawr = state_icp.yawr - state_start.yawr;
    deltas_icp.push_back(delta_icp);
}
void Plausability::setStateOdo(const State &state)
{
    state_odo = state;
    //Delta State Odo t1-t0
    delta_odo.x = state_odo.x - state_start.x;
    delta_odo.y = state_odo.y - state_start.y;
    delta_odo.v = state_odo.v - state_start.v;
    delta_odo.yaw = state_odo.yaw - state_start.yaw;
    delta_odo.yawr = state_odo.yawr - state_start.yawr;
    deltas_odo.push_back(delta_odo);
}
void Plausability::deltaOdoICP()
{
    delta_odo_icp.x = state_odo.x - state_icp.x;
    delta_odo_icp.y = state_odo.y - state_icp.y;
    delta_odo_icp.v = state_odo.v - state_icp.v;
    delta_odo_icp.yaw = state_odo.yaw - state_icp.yaw;
    delta_odo_icp.yawr = state_odo.yawr - state_icp.yawr;
    deltas_odo_icp.push_back(delta_odo_icp);
}

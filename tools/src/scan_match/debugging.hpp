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

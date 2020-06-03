#include <scan_match/debugging.hpp>

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

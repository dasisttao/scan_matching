#pragma once
#include <scan_match/var_defs.hpp>
#include <geometry_msgs/PoseStamped.h>
#include "ros_can_gps_msg/gpsData.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include <numeric>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class UKF
{
public:
  UKF();
  virtual ~UKF(){};

  void Prediction(double delta_t);
  void UpdateMeasurementAruco(const geometry_msgs::PoseStamped &meas_data);
  //void UpdateMeasurementCAN(const ros_can_gps_msg::gpsData &meas_data);
  void UpdateMeasurementCAN2(const vector<double> &meas_datas);
  void UpdateMeasurementLidar(const vector<double> &meas_datas);
  void UpdateMeasurementCAN(const vector<double> &meas_datas);
  void UpdateLidar(const vector<double> &meas_datas);
  void UpdateCAN(const vector<double> &meas_datas);
  bool is_initialized_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time of the state - x_ and P_
  double time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;
  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // measurement noise standard deviation position1 in m
  double std_px_;
  // measurement noise standard deviation position2 in m
  double std_py_;
  // measurement noise standard deviation velocity in m/s
  double std_v_;
  // measurement noise standard deviation angle in rad
  double std_radphi_;
  // measurement noise standard deviation angle change in rad/s
  double std_radphip_;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
};

UKF::UKF()
{

  is_initialized_ = false;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.1;

  //@ measurement noise values are provided by the sensor manufacturer.
  //@ measurement noise standard deviation
  //  position1 in m
  std_px_ = 0.15;
  // position2 in m
  std_py_ = 0.15;
  // vel in m/s
  std_v_ = 0.6; //0.3
  // yaw angle in rad
  std_radphi_ = 0.1; //0.03
  // yaw angle change in rad/s
  std_radphip_ = 0.6; //0.3

  //Laser
  std_laspx_ = 0.15;
  std_laspy_ = 0.15;

  time_us_ = 0;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  weights_ = VectorXd(2 * n_aug_ + 1);

  P_ << 10, 0.03, -0.014, 0, 0,
      0.03, 10, -0.01, 0, 0,
      -0.014, 0, 0.05, 0, 0,
      0, 0, 0, 0, 0,
      0, 0, 0, 0, 0;

  x_ << 0, 0, 0, 0, 0;
}

void UKF::Prediction(double delta_t)
{

  /*****************************************************************************
  *  Generate Sigma Points
  ****************************************************************************/
  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set lambda for non-augmented sigma points
  lambda_ = 3 - n_x_;

  //set first column of sigma point matrix
  Xsig.col(0) = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }

  /*****************************************************************************
  *  Augment Sigma Points
  ****************************************************************************/
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //set lambda for augmented sigma points
  lambda_ = 3 - n_aug_;

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  /*****************************************************************************
  *  Predict Sigma Points
  ****************************************************************************/
  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;
    // ukf_filter.x_(0) = ukf_filter.x_(0) + ukf_filter.x_(2) * dt * cos(ukf_filter.x_(3));
    // ukf_filter.x_(1) = ukf_filter.x_(1) + ukf_filter.x_(2) * dt * sin(ukf_filter.x_(3));
    //avoid division by zero
    if (fabs(yawd) > 0.001)
    {
      // yaw = yaw + 90;
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else
    {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }
    // px_p = p_x + v * delta_t * cos(yaw);
    // py_p = p_y + v * delta_t * sin(yaw);
    double v_p = v;
    double yaw_p = yaw - yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  /*****************************************************************************
  *  Convert Predicted Sigma Points to Mean/Covariance
  ****************************************************************************/

  // set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
  { //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateMeasurementLidar(const vector<double> &meas_datas)
{

  VectorXd z(2);

  z << meas_datas[0], meas_datas[1];

  MatrixXd H(2, 5);
  H << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0;
  double temp_yaw = x_(3);
  double temp_yawr = x_(4);
  MatrixXd R(2, 2);
  double std_laspx_2 = 0.05;
  R << std_laspx_2 * std_laspx_2, 0,
      0, std_laspx_2 * std_laspx_2;
  // std_v_ = 0.3;       //0.3
  // std_radphi_ = 0.03; //0.03
  // std_radphip_ = 0.1; //0.1
  // double std_xy = 0.2;

  // R << std_xy * std_xy, 0, 0, 0, 0,
  //     0, std_xy * std_xy, 0, 0, 0,
  //     0, 0, std_v_ * std_v_, 0, 0,
  //     0, 0, 0, std_radphi_ * std_radphi_, 0,
  //     0, 0, 0, 0, std_radphip_ * std_radphip_;

  VectorXd y = z - H * x_;
  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + (K * y);
  x_(3) = temp_yaw;
  x_(4) = temp_yawr;

  P_ = (MatrixXd::Identity(5, 5) - K * H) * P_;
}

void UKF::UpdateMeasurementCAN2(const vector<double> &meas_datas)
{

  VectorXd z(3);

  z << meas_datas[0],
      x_(3), // HIER MINUS! WIESO AUCH IMMER
      meas_datas[1];

  MatrixXd H(3, 5);
  H << 0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;

  MatrixXd R(3, 3);

  std_v_ = 0.3;       //0.3
  std_radphi_ = 0.03; //0.03
  std_radphip_ = 0.1; //0.1

  R << std_v_ * std_v_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radphip_ * std_radphip_;

  VectorXd y = z - H * x_;

  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + (K * y);
  x_(3) = atan2(sin(x_(3)), cos(x_(3)));

  P_ = (MatrixXd::Identity(5, 5) - K * H) * P_;
}

void UKF::UpdateMeasurementCAN(const vector<double> &meas_datas)
{

  VectorXd z(2);

  z << meas_datas[0],
      meas_datas[1];

  MatrixXd H(2, 5);
  H << 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1;

  MatrixXd R(2, 2);

  std_v_ = 0.01;      //0.3
  std_radphi_ = 0.03; //0.03
  std_radphip_ = 0.1; //0.1

  R << std_v_ * std_v_, 0,
      0, std_radphip_ * std_radphip_;

  VectorXd y = z - H * x_;
  MatrixXd S = H * P_ * H.transpose() + R;
  MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + (K * y);

  P_ = (MatrixXd::Identity(5, 5) - K * H) * P_;
}

void UKF::UpdateLidar(const vector<double> &meas_datas)
{

  int n_z = 3; // Number of Laser Measurements
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double yaw = Xsig_pred_(3, i);

    // measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
    Zsig(2, i) = yaw;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(2) = atan2(sin(z_diff(2)), cos(z_diff(2)));
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0, 0,
      0, std_laspy_ * std_laspy_, 0,
      0, 0, 0.05 * 0.05;
  S = S + R;

  //--------------------------------------UKF Update
  VectorXd x_out = VectorXd(5);
  MatrixXd P_out = MatrixXd(5, 5);
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    z_diff(2) = atan2(sin(z_diff(2)), cos(z_diff(2)));
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z = VectorXd(n_z);
  z << meas_datas[0], meas_datas[1], meas_datas[2];
  VectorXd z_diff = z - z_pred;
  z_diff(2) = atan2(sin(z_diff(2)), cos(z_diff(2)));
  // x_(3) = meas_datas[2]; // Hack, weil normalisieren nicht richtig funktioniert
  x_ = x_ + K * z_diff;

  P_ = P_ - K * S * K.transpose();
}

void UKF::UpdateCAN(const vector<double> &meas_datas)
{
  int n_z = 2; // Number of Radar Measurements
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points

    // measurement model
    Zsig(0, i) = Xsig_pred_(2, i); //speed
    Zsig(1, i) = Xsig_pred_(4, i); //yaw_rate
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;
  S = S + R;

  //--------------------------------------UKF Update
  VectorXd x_out = VectorXd(5);
  MatrixXd P_out = MatrixXd(5, 5);
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z = VectorXd(n_z);
  z << meas_datas[0], meas_datas[1];
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;

  P_ = P_ - K * S * K.transpose();
}

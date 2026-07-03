#include "oni_nav_controller/state_estimator.hpp"

#include <cmath>

namespace oni_nav_controller
{

StateEstimator::StateEstimator(const StateEstimatorParams & params)
: params_(params)
{
  P_.setIdentity();
  P_ *= 1.0;
}

void StateEstimator::reset(const StateVector & x0)
{
  x_ = x0;
  x_(THETA) = normalize_angle(x_(THETA));
  P_.setIdentity();
  initialized_ = true;
}

void StateEstimator::predict(double dt, const ImuSample & imu)
{
  if (!initialized_ || dt <= 0.0) {
    return;
  }

  const double x = x_(X);
  const double y = x_(Y);
  const double theta = x_(THETA);
  const double vx = x_(VX);
  const double vy = x_(VY);
  const double omega = x_(OMEGA);

  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);

  StateVector x_pred;
  x_pred(X) = x + (vx * cos_t - vy * sin_t) * dt;
  x_pred(Y) = y + (vx * sin_t + vy * cos_t) * dt;
  x_pred(THETA) = normalize_angle(theta + omega * dt);
  x_pred(VX) = vx + imu.ax * dt;
  x_pred(VY) = vy + imu.ay * dt;
  x_pred(OMEGA) = imu.omega;

  StateMatrix F = StateMatrix::Identity();
  F(X, THETA) = (-vx * sin_t - vy * cos_t) * dt;
  F(X, VX) = cos_t * dt;
  F(X, VY) = -sin_t * dt;
  F(Y, THETA) = (vx * cos_t - vy * sin_t) * dt;
  F(Y, VX) = sin_t * dt;
  F(Y, VY) = cos_t * dt;
  F(THETA, OMEGA) = dt;

  StateMatrix Q = StateMatrix::Zero();
  Q(X, X) = params_.process_noise_pos;
  Q(Y, Y) = params_.process_noise_pos;
  Q(THETA, THETA) = params_.process_noise_theta;
  Q(VX, VX) = params_.process_noise_vel;
  Q(VY, VY) = params_.process_noise_vel;
  Q(OMEGA, OMEGA) = params_.process_noise_omega;

  x_ = x_pred;
  x_(THETA) = normalize_angle(x_(THETA));
  P_ = F * P_ * F.transpose() + Q;
}

void StateEstimator::update_odom(const Velocity3 & v_odom)
{
  if (!initialized_) {
    return;
  }

  OdomMeasurement z;
  z << v_odom.vx, v_odom.vy, v_odom.omega;

  OdomMeasurement z_pred;
  z_pred << x_(VX), x_(VY), x_(OMEGA);

  Eigen::Matrix<double, ODOM_MEAS_DIM, STATE_DIM> H =
    Eigen::Matrix<double, ODOM_MEAS_DIM, STATE_DIM>::Zero();
  H(0, VX) = 1.0;
  H(1, VY) = 1.0;
  H(2, OMEGA) = 1.0;

  Eigen::Matrix<double, ODOM_MEAS_DIM, ODOM_MEAS_DIM> R =
    Eigen::Matrix<double, ODOM_MEAS_DIM, ODOM_MEAS_DIM>::Identity();
  R(0, 0) = params_.meas_noise_vel;
  R(1, 1) = params_.meas_noise_vel;
  R(2, 2) = params_.meas_noise_omega;

  const OdomMeasurement innovation = z - z_pred;
  const Eigen::Matrix<double, ODOM_MEAS_DIM, ODOM_MEAS_DIM> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, STATE_DIM, ODOM_MEAS_DIM> K = P_ * H.transpose() * S.inverse();

  x_ += K * innovation;
  x_(THETA) = normalize_angle(x_(THETA));
  P_ = (StateMatrix::Identity() - K * H) * P_;
}

void StateEstimator::update_odom(const Velocity3 & v_odom, double x, double y, double theta)
{
  update_odom(v_odom);

  if (!initialized_) {
    return;
  }

  x_(X) = x;
  x_(Y) = y;
  x_(THETA) = normalize_angle(theta);
}

const StateVector & StateEstimator::get_state() const
{
  return x_;
}

bool StateEstimator::is_initialized() const
{
  return initialized_;
}

double StateEstimator::normalize_angle(double theta)
{
  while (theta > M_PI) {
    theta -= 2.0 * M_PI;
  }
  while (theta < -M_PI) {
    theta += 2.0 * M_PI;
  }
  return theta;
}

}  // namespace oni_nav_controller

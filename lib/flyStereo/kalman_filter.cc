#include "flyStereo/kalman_filter.h"

#include <iostream>

KalmanFilter::KalmanFilter(const Eigen::MatrixXd &initial_state)
    : state_(initial_state),
      covariance_(num_states, num_states),
      f_(num_states, num_states),
      q_(num_states, num_states),
      h_(num_measurements, num_states),
      r_(num_measurements, num_measurements) {
  if (initial_state.rows() != num_states) {
    throw std::invalid_argument("Bad initial state vector length!");
  }
  Init();
}

void KalmanFilter::Init() {
  covariance_ << Eigen::Matrix<double, num_states, num_states>::Identity() * 100;
  // covariance_ << Eigen::Matrix<double, num_states, num_states>::Zero();

  // TODO: don't hardcode these
  f_ << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  dt_ = 0.2;
  double sigma_a = 1.0;
  h_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  r_ << 0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001;

  Eigen::Vector2d g;
  g << 0.5 * dt_ * dt_, dt_;
  q_ << Eigen::Matrix<double, num_states, num_states>::Zero();
  q_.block<2, 2>(0, 0) = g * g.transpose() * (sigma_a * sigma_a);
  q_.block<2, 2>(2, 2) = g * g.transpose() * (sigma_a * sigma_a);
  q_.block<2, 2>(4, 4) = g * g.transpose() * (sigma_a * sigma_a);
}

void KalmanFilter::Predict(double dt) {
  // Check if the user ommited the dt argument and fill it with the default value
  if (dt < 0) {
    dt = dt_;
  }

  Eigen::Matrix<double, num_states, 1> u = Eigen::Matrix<double, num_states, 1>::Zero(num_states);
  int i;
  for (i = 0; i < num_states / 2; i++) {
    u(2 * i) = state_(2 * i + 1);
  }

  // Apply the prediction
  state_ = state_ + u * dt;

  // Apply the prediction
  // state_ = (f_ * dt + Eigen::Matrix<double, num_states, num_states>::Identity()) * state_;
  // state_ = state_ + (f_ * dt);
  Eigen::Matrix<double, num_states, num_states> mod_f =
      Eigen::Matrix<double, num_states, num_states>::Identity() + (f_ * dt);
  covariance_ = mod_f * covariance_ * mod_f.transpose() + q_;
}

int KalmanFilter::Measure(const Eigen::Matrix<double, num_measurements, 1> &z) {
  if (z.rows() != num_measurements) {
    std::cerr << "Bad measurement vector length!" << std::endl;
    return -1;
  }

  // with Z as the given measurement
  Eigen::Matrix<double, num_measurements, 1> y = z - (h_ * state_);

  Eigen::Matrix<double, num_measurements, num_measurements> s = h_ * covariance_ * h_.transpose() + r_;

  // optimal kalman gain
  Eigen::Matrix<double, num_states, num_measurements> k = covariance_ * h_.transpose() * s.inverse();

  // update state
  state_ = state_ + (k * y);

  // update covariance
  covariance_ = (Eigen::Matrix<double, num_states, num_states>::Identity() - (k * h_)) * covariance_;

  return 0;
}

Eigen::Matrix<double, 1, num_states> KalmanFilter::GetState() { return state_; }

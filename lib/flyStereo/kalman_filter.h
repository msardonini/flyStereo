#pragma once

#include <memory>

#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"

constexpr int num_states = 6;
constexpr int num_measurements = 3;

class KalmanFilter {
 public:
  /**
   * @brief      Constructs the object.
   *
   * @param[in]  initial_state  The initial state
   * @param[in]  delta_t        The delta T
   */
  KalmanFilter(const Eigen::MatrixXd &initial_state = Eigen::Matrix<double, num_states, 1>::Zero());

  /** Initializes the required matrices
   *
   */
  void Init();

  /** Perform a prediction update with a defined time step for a position and
   * velocity model
   *
   * @param[in]  dt    The delta T
   *
   * @return     Returns the state of the prediction step
   */
  void Predict(double dt = -1);

  /** Perform a measurement update with default time step
   *
   * @param[in]  Z     The measurement
   *
   * @return     Returns 0 on success, -1 on error
   */
  int Measure(const Eigen::Matrix<double, num_measurements, 1> &z);

  // Accessors
  Eigen::Matrix<double, 1, num_states> GetState();

 private:
  // Private methods --------------------------
  void UpdateTimeStep(double _dt_);

  Eigen::Matrix<double, num_states, 1> state_;
  Eigen::Matrix<double, num_states, num_states> covariance_;

  // state function - relationship between the state variables
  Eigen::Matrix<double, num_states, num_states> f_;
  Eigen::Matrix<double, num_states, num_states> q_;

  // measurement function - reflect the fact that we observe the positions but
  // not velocities
  Eigen::Matrix<double, num_measurements, num_states> h_;

  // measurement noise
  Eigen::Matrix<double, num_measurements, num_measurements> r_;

  // timestep
  double noise_ = 0.1;
  double dt_;
};

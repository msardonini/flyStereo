
#ifndef LIB_INCLUDE_FLY_STEREO_KALMAN_FILTER_H_
#define LIB_INCLUDE_FLY_STEREO_KALMAN_FILTER_H_

#include <memory>
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"

constexpr int num_states = 6;
constexpr int num_measurements = 3;

class KalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KalmanFilter() = delete;

  /**
   * @brief      Constructs the object.
   *
   * @param[in]  initial_state  The initial state
   * @param[in]  delta_t        The delta T
   */
  KalmanFilter(const YAML::Node &input_params,
  const Eigen::MatrixXd &initial_state = Eigen::Matrix<double, num_states, 1>::Zero());

  /** Initializes the required matrices
   *
   */
  void Init(const YAML::Node &input_params);

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
  int Measure(const Eigen::Matrix<double, 1, num_measurements> &z);

  // Accessors
  Eigen::MatrixXd GetState();

 private:
  // Private methods --------------------------
  void UpdateTimeStep(double _dt_);

  Eigen::MatrixXd state_;
  Eigen::MatrixXd covariance_;

  // state function - relationship between the state variables
   Eigen::MatrixXd f_;
  Eigen::MatrixXd q_;

  // measurement function - reflect the fact that we observe the positions but
  // not velocities
  Eigen::MatrixXd h_;

  // measurement noise
  Eigen::MatrixXd r_;

  // timestep
  double noise_ = 0.1;
  double dt_;
};

#endif  // LIB_INCLUDE_FLY_STEREO_KALMAN_FILTER_H_
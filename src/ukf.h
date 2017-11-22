#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  UKF();
  virtual ~UKF();

  /**
   * ProcessMeasurement computes the next estimation of the state.
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage &measurement);

private:
  // Minimum state value
  static constexpr float kMinStateValue = 0.0001;

  // Initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;
  
  // If this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // If this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // State covariance matrix
  MatrixXd P_;

  // Predicted sigma points matrix
  MatrixXd Xsig_pred_;

  MatrixXd R_radar_;
  MatrixXd R_lidar_;

  // Time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  double previous_timestamp_;

  // Normalizes the angle at the position pos of the vector x.
  void AngleNormalization(VectorXd *x, int pos);

  void ComputeSigmaPoints(const VectorXd &x_aug, const MatrixXd &P_aug, double delta_t);

  void ComputeSigmaPoints(const VectorXd &x, double delta_t, int column);

  // Initializes the state depending on the measurement received.
  void Initialize(const MeasurementPackage &measurement);

  void InitializeAugmentedX(VectorXd *x_aug);
  
  void InitializeAugmentedP(MatrixXd *P_aug);

  // Returns true if we are processing the measurement given its source.
  bool IsMeasurementProcessable(const MeasurementPackage &measurement);

  void PredictState();

  void PredictCovariance();

  //InitializeWeights initializes the weights.
  void InitializeWeights();

  /**
   * MaybeCorrectState ensures that the values of the state position are not
   * too small to cause division by 0.
   */
  void MaybeCorrectState();

  /**
   * Predict predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Predict(const double delta_t);

  
  /**
   * Update handles the call to the different update methods
   * @param measurement The mesasurement at k+1
   */
  void Update(const MeasurementPackage &measurement);

  void Update(const MeasurementPackage &measurement, MatrixXd Zsig);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param measurement The measurement at k+1
   */
  MatrixXd UpdateLidar(const MeasurementPackage &measurement);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param measurement The measurement at k+1
   */
  MatrixXd UpdateRadar(const MeasurementPackage &measurement);

  MatrixXd MeasurementCovariance(const MatrixXd &Zsig, const VectorXd &z_pred);

  MatrixXd MeasurementNoiseCovariance(MeasurementPackage::SensorType sensor_type);
  
  MatrixXd CrossCorrelation(const MatrixXd &Zsig, const VectorXd &z_pred);
  
  MatrixXd KalmanGain(const MatrixXd &Zsig, const MatrixXd &S);

};

#endif /* UKF_H */

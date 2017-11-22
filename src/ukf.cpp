#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;
  use_laser_ = true;
  use_radar_ = true;

  x_ = VectorXd(5);
  P_ = MatrixXd(5, 5);

  std_a_ = 1.5;
  std_yawdd_ = 0.5;
  std_laspx_ = 0.15;
  std_laspy_ = 0.15;
  std_radr_ = 0.3;
  std_radphi_ = 0.03;
  std_radrd_ = 0.3;

  n_x_ = x_.size();
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  weights_ = VectorXd(2*n_aug_+1);

  InitializeWeights();

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

}

UKF::~UKF() {}

void UKF::InitializeWeights() {
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1; i<weights_.size(); i++) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }
}

void UKF::ProcessMeasurement(const MeasurementPackage &measurement) {
  if (!IsMeasurementProcessable(measurement)) {
    return;
  }
  if (!is_initialized_) { 
    Initialize(measurement);
    return;
  }
  float dt_s = (measurement.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement.timestamp_;
  Predict(dt_s);
  Update(measurement);
}

bool UKF::IsMeasurementProcessable(const MeasurementPackage &measurement) {
  return measurement.sensor_type_ == MeasurementPackage::RADAR && use_radar_ 
      || measurement.sensor_type_ == MeasurementPackage::LASER && use_laser_; 
}

void UKF::Initialize(const MeasurementPackage &measurement) {
  if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
    float rho = measurement.raw_measurements_[0];
    float phi = measurement.raw_measurements_[1];
    float rho_dot = measurement.raw_measurements_[2];
    // px, py, v, 
    x_ << rho * cos(phi),
          rho * sin(phi),
          rho_dot,
          0,
          0;
  } else {
    x_ << measurement.raw_measurements_[0],
          measurement.raw_measurements_[1],
          0,
          0,
          0;
  }
  P_ = MatrixXd::Identity(5,5);
  MaybeCorrectState();
  previous_timestamp_ = measurement.timestamp_;
  is_initialized_ = true;
}

void UKF::MaybeCorrectState() {
  if (fabs(x_(0)) < kMinStateValue) {
    x_(0) = kMinStateValue;
  }
  if (fabs(x_(1)) < kMinStateValue) {
    x_(1) = kMinStateValue;
  }
}

void UKF::Predict(const double delta_t) {
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  InitializeAugmentedX(&x_aug);
  InitializeAugmentedP(&P_aug);
  ComputeSigmaPoints(x_aug, P_aug, delta_t);
  PredictState();
  PredictCovariance();
}

void UKF::InitializeAugmentedX(VectorXd *x_aug) {
  if (x_aug->size() != n_aug_) {
    // TODO: handle error.
    return;
  }
  x_aug->fill(0.0);
  x_aug->head(n_x_) = x_;
}

void UKF::InitializeAugmentedP(MatrixXd *P_aug) {
  P_aug->fill(0.0);
  P_aug->topLeftCorner(5,5) = P_;
  (*P_aug)(n_x_, n_x_) = std_a_ * std_a_;
  (*P_aug)(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;
}

void UKF::ComputeSigmaPoints(const VectorXd &x_aug, const MatrixXd &P_aug,
    double delta_t) {
  double sqrt_lambda = sqrt(lambda_ + n_aug_);
  MatrixXd L = P_aug.llt().matrixL();
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  Xsig_aug.col(0) = x_aug;

  for (int i=0; i<n_aug_; i++) {
    Xsig_aug.col(i+1) = x_aug + sqrt_lambda * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt_lambda * L.col(i);
  }
  for (int i=0; i<2*n_aug_+1; i++) {
    ComputeSigmaPoints(Xsig_aug.col(i), delta_t, i);
  }
}

void UKF::ComputeSigmaPoints(const VectorXd &x, double delta_t, int column) {
  double p_x = x(0);   
  double p_y = x(1);
  double v = x(2);
  double yaw = x(3);
  double yawd = x(4);
  double nu = x(5);
  double nu_yawdd = x(6);

  // First term of the Taylor expansion of predicted yaw: yaw + delta_t*yaw_d
  double first_yaw_p = yaw + delta_t * yawd;
  if (fabs(yawd) > kMinStateValue) {
    p_x += (sin(first_yaw_p) - sin(yaw)) * v/yawd;
    p_y += (cos(yaw) - cos(first_yaw_p)) * v/yawd;
  } else {
    p_x += v * delta_t * cos(yaw);
    p_y += v * delta_t * sin(yaw);
  }

  // Adding noise:
  Xsig_pred_(0,column) = p_x + 0.5 * nu * delta_t * delta_t * cos(yaw);
  Xsig_pred_(1,column) = p_y + 0.5 * nu * delta_t * delta_t * sin(yaw);
  Xsig_pred_(2,column) = v + nu * delta_t;
  Xsig_pred_(3,column) = first_yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
  Xsig_pred_(4,column) = yawd + nu_yawdd * delta_t;
}

void UKF::PredictState() {
  x_ = Xsig_pred_ * weights_; 
}

void UKF::PredictCovariance() {
  P_.fill(0.0);
  VectorXd x_diff;
  for (int i=0; i<2*n_aug_+1; i++) {
    x_diff = Xsig_pred_.col(i) - x_;
    AngleNormalization(&x_diff, 3);
    MatrixXd temp = x_diff * x_diff.transpose();
    P_ += weights_(i) * temp;
  }
}

void UKF::AngleNormalization(VectorXd *x, int pos) {
  while ((*x)[pos] > M_PI || (*x)[pos] < -M_PI) {
    if ((*x)[pos] > M_PI) {
      (*x)[pos] -= M_PI;
    } else {
      (*x)[pos] += M_PI;
    }
  }
}

void UKF::Update(const MeasurementPackage &measurement) {
  MatrixXd Zsig;
  if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
    Zsig = UpdateRadar(measurement);
    Update(measurement, Zsig);
  } else {
    Zsig = UpdateLidar(measurement);
    Update(measurement, Zsig);
  }
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
MatrixXd UKF::UpdateRadar(const MeasurementPackage &measurement) {
  // Radar measurement is 3D: r, phi, r_dot
  int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  for (int i=0; i<2*n_aug_+1; i++) {
    double p_x_v_x = Xsig_pred_(0,i) * cos(Xsig_pred_(3,i)) * Xsig_pred_(2,i);
    double p_y_v_y = Xsig_pred_(1,i) * sin(Xsig_pred_(3,i)) * Xsig_pred_(2,i);

    Zsig(0,i) = sqrt(Xsig_pred_(0,i)*Xsig_pred_(0,i) + Xsig_pred_(1,i)*Xsig_pred_(1,i));
    Zsig(1,i) = atan2(Xsig_pred_(1,i), Xsig_pred_(0,i));
    Zsig(2,i) = (p_x_v_x + p_y_v_y) / Zsig(0,i);
  }
  return Zsig;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
MatrixXd UKF::UpdateLidar(const MeasurementPackage &measurement) {
  return Xsig_pred_.block(0, 0, 2, 2*n_aug_+1);
}

void UKF::Update(const MeasurementPackage &measurement, MatrixXd Zsig) {
  int n_rows = Zsig.rows();

  VectorXd z_pred = Zsig * weights_;

  MatrixXd S = MeasurementCovariance(Zsig, z_pred);
  MatrixXd R = MeasurementNoiseCovariance(measurement.sensor_type_);
  S += R;

  MatrixXd Tc = CrossCorrelation(Zsig, z_pred);
  MatrixXd K = KalmanGain(Tc, S);

  VectorXd z_residual = measurement.raw_measurements_ - z_pred;
  if (measurement.sensor_type_ == MeasurementPackage::RADAR) {
    AngleNormalization(&z_residual, 1);
  }
  x_ += K * z_residual;
  P_ = P_ - K * S * K.transpose();
}

MatrixXd UKF::MeasurementCovariance(const MatrixXd &Zsig, const VectorXd &z_pred) {
  int n_rows = Zsig.rows();
  MatrixXd S = MatrixXd::Zero(n_rows, n_rows);
  VectorXd z_diff;

  for (int i=0; i<2*n_aug_+1; i++) {
    z_diff = Zsig.col(i) - z_pred;  
    AngleNormalization(&z_diff, 1);
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  return S;
}

MatrixXd UKF::MeasurementNoiseCovariance(MeasurementPackage::SensorType sensor_type) {
  if (sensor_type == MeasurementPackage::RADAR) {
    return R_radar_;
  }
  return R_lidar_;
}

MatrixXd UKF::CrossCorrelation(const MatrixXd &Zsig, const VectorXd &z_pred) {
  int n_rows = Zsig.rows();
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_rows);
  VectorXd z_diff;
  VectorXd x_diff;

  for (int i=0; i<2*n_aug_+1;i++) {
    z_diff = Zsig.col(i) - z_pred;
    // Only normalize when radar
    if (n_rows == 3) {
      AngleNormalization(&z_diff, 1);
    }
    x_diff = Xsig_pred_.col(i) - x_;
    AngleNormalization(&x_diff, 3);
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
  
  return Tc;
}

MatrixXd UKF::KalmanGain(const MatrixXd &Tc, const MatrixXd &S) {
  return Tc * S.inverse();
}

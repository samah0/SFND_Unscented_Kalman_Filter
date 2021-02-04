#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
  void AugmentedSigmaPoints();
  void SigmaPointPrediction(double delta_t);
  void PredictMeanAndCovariance();
  void PredictRadarMeasurement();
  void UpdateState(MeasurementPackage meas_package);
  
  void PredictLaserMeasurement();                 
  void UpdateState_Laser(MeasurementPackage meas_package); 

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;
  
  // sigma points matrix
  //Eigen::MatrixXd Xsig_;
  
  // augmented sigma points matrix
  Eigen::MatrixXd Xsig_aug_;
  
  // mean predicted measurement: RADAR
  Eigen::VectorXd z_pred_;
  
  // mean predicted measurement: LASER
  Eigen::MatrixXd z_pred_laser_;
  
  // matrix with sigma points in measurement space: RADAR
  Eigen::MatrixXd Zsig_;
  
  // matrix with sigma points in measurement space: LASER
  Eigen::MatrixXd Zsig_laser_;
  
  // measurement covariance matrix: RADAR
  Eigen:: MatrixXd S_;
  
  // measurement covariance matrix; LASER
  Eigen::MatrixXd S_laser_;
  
  // measurement noise covariance matrix: RADAR
  Eigen::MatrixXd R_radar_;
  
  // measurement noise covariance matrix: LASER
  Eigen::MatrixXd R_laser_;
  
  // Weights of sigma points
  Eigen::VectorXd weights_;
 
  // time when the state is true, in us
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

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;
  
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z_;
  
};

#endif  // UKF_H

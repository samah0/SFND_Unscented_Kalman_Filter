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

  
  void GenerateSigmaPoints(Eigen::MatrixXd* Xsig_out);
  void AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out);
  void SigmaPointPrediction(Eigen::MatrixXd* Xsig_out, double delta_t);
  void PredictMeanAndCovariance(Eigen::VectorXd* x_pred, 
                                Eigen::MatrixXd* P_pred);
  void PredictRadarMeasurement(Eigen::VectorXd* z_out, 
                               Eigen::MatrixXd* S_out);
  void UpdateState(Eigen::VectorXd* x_out, 
                   Eigen::MatrixXd* P_out,MeasurementPackage meas_package);
  
  void PredictLaserMeasurement();                 
  void UpdateState_Laser(Eigen::VectorXd* x_out, 
                   Eigen::MatrixXd* P_out, MeasurementPackage meas_package);                 

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
  Eigen::MatrixXd Xsig_;
  
  // augmented sigma points matrix
  Eigen::MatrixXd Xsig_aug_;
  
  // mean predicted measurement
  Eigen::VectorXd z_pred_;
  
  // matrix with sigma points in measurement space
  Eigen::MatrixXd Zsig_;
  
  Eigen::MatrixXd z_pred_laser_;
  Eigen::MatrixXd Zsig_laser_;
  
  // measurement covariance matrix S
  Eigen:: MatrixXd S_;
  Eigen::MatrixXd S_laser_;
  
  // measurement noise covariance matrix
  //Eigen::MatrixXd R;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;

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
  
  double lambda;
  
  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z_;
  
  // previous timestamp
  long long previous_timestamp_;
};

#endif  // UKF_H
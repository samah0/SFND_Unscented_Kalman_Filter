#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  
  is_initialized_ = false;
  time_us_ = 0;

  // initial state vector
  x_ = VectorXd(5);
 
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  
  std_yawdd_ = 5.; 
  
  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;
  
  // set measurement dimension, radar can measure r, phi, and r_dot
  n_z_ = 3;

   // initial state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_ = VectorXd(n_x_).setZero();

  // initial state covariance matrix
  MatrixXd P_ = MatrixXd(n_x_, n_x_).setZero();

  // initial predicted sigma points matrix
  MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1).setZero();
 
  // initial sigma points matrix
  //Xsig_ = MatrixXd(n_x_, 2 * n_x_ + 1).setZero();
  
  // initial augmented sigma points matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1).setZero();
  
  // mean predicted measurement for Radar
  z_pred_ = VectorXd(n_z_).setZero();
  
  // matrix with sigma points in measurement space: Radar
  Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1).setZero();
  
  // mean predicted measurement for Lidar
  z_pred_laser_ = VectorXd(2).setZero();
  
  // matrix with sigma points in measurement space: Lidar
  Zsig_laser_ = MatrixXd(2, 2 * n_aug_ + 1).setZero();


  // measurement covariance matrix: RADAR 
  S_ = MatrixXd(n_z_,n_z_).setZero();
  
  // measurement covariance matrix: LASER
  S_laser_ = MatrixXd(2,2).setZero();
  
  // measurement noise covariance matrix: RADAR
  R_radar_ = MatrixXd(3, 3);
  
  // measurement noise covariance matrix:LASER
  R_laser_= MatrixXd(2, 2);
  
  // initialize spreading parameter
  lambda_ = 3 - n_aug_; 
  
  // initial weights of sigma points
  weights_ = VectorXd(2*n_aug_+1).setZero();
  
  // set vector for weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  double weight = 0.5/(lambda_+n_aug_);
  weights_(0) = weight_0;

  for (int i=1; i<2*n_aug_+1; ++i) {  
    weights_(i) = weight;
  }
 
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
     /**
   * Initialization
   */
  if (!is_initialized_) {
      
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];
      
      double x= rho * cos(phi);
      double y= rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx*vx + vy*vy);
      double yaw = atan2(vy,vx);
      
      // set the state with the initial location and velocity
      x_.fill(0);
      x_ << x, y, v, yaw, 0;
      //x_ << x, y, v, 0, 0;
      /*
      // Initialize the state covariance matrix P
      P_.fill(0);
      P_ << std_radr_*std_radr_, 0, 0, 0, 0,
            0, std_radr_*std_radr_, 0, 0, 0,
            0, 0, std_radrd_*std_radrd_, 0, 0,
            0, 0, 0, std_radphi_*std_radphi_, 0,
            0, 0, 0, 0, std_radrd_*std_radrd_; 
      */
 
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      // set the state with the initial location and zero velocity
      x_.fill(0);
      x_ << meas_package.raw_measurements_[0], 
            meas_package.raw_measurements_[1],
            0, 
            0, 
            0;
       /*    
       // Initialize the state covariance matrix P
       P_.fill(0);
       P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
          0, std_laspy_*std_laspy_, 0, 0, 0,
          0, 0, 0.5, 0, 0,
          0, 0, 0, 0.9, 0,
          0, 0, 0, 0, 0.9; */
    }
    
       P_.fill(0);
       /* // works for last car
       P_ << std_laspx_*std_laspx_,0,0,0,0,
          0,std_laspy_*std_laspy_,0,0,0,
          0, 0, 0.1, 0, 0,
          0, 0, 0, 0.1, 0,
          0, 0, 0, 0, 0.1; 
       */
             
      P_ << 0.05, 0, 0, 0, 0,
            0, 0.05, 0, 0, 0,
            0, 0, std_radrd_*std_radrd_, 0, 0,
            0, 0, 0, std_radphi_*std_radphi_, 0,
            0, 0, 0, 0, std_radrd_*std_radrd_;        
            
      
     // initialize measurement noise covariance matrix
     R_radar_ <<  std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
    
     //measurement covariance matrix - laser
     R_laser_ << std_laspx_, 0,
                 0, std_laspy_;
               
    // previous timestamp
    time_us_ = meas_package.timestamp_; 
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  /**
   * Prediction
   */

  // compute the time elapsed between the current and previous measurements
  // delta_t - expressed in seconds
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_; // Time is measured in seconds.
  Prediction(delta_t);
  
  /**
   * Update
   */
 
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
  
   // Radar updates
   UpdateRadar(meas_package);
    
  } else {
    
    // Laser updates
    UpdateLidar(meas_package);
    
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
 
   // Step1: Generate Augmented Sigma Points 
   AugmentedSigmaPoints();
  
   // Step2: Predict Sigma Points
   SigmaPointPrediction(delta_t);
   
   // Step3: Predict the state, and the state covariance matrix
   PredictMeanAndCovariance();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
   
   // Step1: Predict Lidar measurment
   PredictLaserMeasurement();
   
   // Step2: Update State and covariance, P_
   UpdateState_Laser(meas_package);
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
   
   // Step1: Transform the predicted state into measurment space (predicted measurment space)
   PredictRadarMeasurement();
   
   // Step2: Update State and covariance, P_
   UpdateState(meas_package);
}

void UKF::AugmentedSigmaPoints() {
  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  // create augmented mean state
  x_aug.head(5) = x_;  
  x_aug(5) = 0;
  x_aug(6) = 0;

   
  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  
  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  
  // create augmented sigma points
  
  // set first column of sigma point matrix
  Xsig_aug.fill(0);
  Xsig_aug.col(0) = x_aug;

  // set remaining sigma points
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  
  // print result
  //std::cout << "Xsig_aug = " << std::endl << Xsig_aug_ << std::endl;

  // write result
  Xsig_aug_ = Xsig_aug;
}

void UKF::SigmaPointPrediction(double delta_t) {
  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_+ 1);

  Xsig_pred.fill(0); 
  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; ++i) {
    
    // extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
    
  }

  // print result
  //std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  Xsig_pred_ = Xsig_pred;
}


void UKF::PredictMeanAndCovariance() {
  
  // create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_+ 1; ++i) {  // iterate over sigma points
    x = x + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  /*
  // print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;
  */
  // write result
  x_ = x;
  P_ = P;
}

void UKF::PredictRadarMeasurement() {
  Zsig_.fill(0);
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig_(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig_(1,i) = atan2(p_y,p_x);                                // phi
    Zsig_(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred_.fill(0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
  }

  S_.fill(0);
  for (int i = 0; i < 2 * n_aug_+ 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  S_ = S_ + R_radar_;
}


void UKF::UpdateState(MeasurementPackage meas_package) { 
  // create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  z.fill(0);
  double rho = meas_package.raw_measurements_[0];
  double phi = meas_package.raw_measurements_[1];
  double rho_dot = meas_package.raw_measurements_[2];
  
  z << rho,       // rho in m
       phi,       // phi in rad
       rho_dot;   // rho_dot in m/s

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_+ 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig_.col(i) - z_pred_;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S_.inverse();

  // residual
  VectorXd z_diff = z - z_pred_;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_*K.transpose();
}

void UKF::PredictLaserMeasurement() {

  Zsig_laser_.fill(0);
  
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
 
    // measurement model
    Zsig_laser_(0,i) = p_x;
    Zsig_laser_(1,i) = p_y;
  }

  // mean predicted measurement
  z_pred_laser_.fill(0);
  for (int i=0; i < 2*n_aug_+1; ++i) {
    z_pred_laser_ = z_pred_laser_ + weights_(i) * Zsig_laser_.col(i);
  }

  S_laser_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_+ 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig_laser_.col(i) - z_pred_laser_;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_laser_ = S_laser_ + weights_(i) * z_diff * z_diff.transpose();
  
  }
  // add measurement noise covariance matrix
  S_laser_ = S_laser_ + R_laser_;
}


void UKF::UpdateState_Laser(MeasurementPackage meas_package) { 
  // create example vector for incoming radar measurement
  VectorXd z = VectorXd(2);
  z.fill(0.0);
  double x = meas_package.raw_measurements_[0];
  double y = meas_package.raw_measurements_[1];
   
  z << x,
       y;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, 2);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_+ 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig_laser_.col(i) - z_pred_laser_;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S_laser_.inverse();
  
  // residual
  VectorXd z_diff = z - z_pred_laser_;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  double NIS = z_diff.transpose()*S_laser_.inverse()*z_diff;
  //std::cout << "\nLASER NIS: " << NIS << std::endl;
  
  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_laser_*K.transpose();
 
}


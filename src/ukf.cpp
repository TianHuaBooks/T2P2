#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define NEAR_ZERO 0.0001


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix to an identity matrix
  P_ = MatrixXd::Identity(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

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

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  n_z_ = 3;
    
  //set measurement dimension, laser can measure x and y
  n_l_ = 2;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  // # of sigma pts
  n_sigma_ = 2 * n_aug_+1;
  Xsig_pred_ = Eigen::MatrixXd(n_aug_, n_sigma_);
    
  //set vector for weights
  weights_ = VectorXd(n_sigma_);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  double weight = 0.5/(n_aug_+lambda_);
  for (int i=1; i<n_sigma_; i++) {  //2n+1 weights
    weights_(i) = weight;
  }
}

UKF::~UKF() {}

// Calc augmented Sigma Pts
void UKF::AugmentedSigmaPoints(MatrixXd& Xsig_aug) {
    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    //create augmented mean state
    int n_x_1 = n_x_ + 1;
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0;
    x_aug(n_x_1) = 0;
    
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_;
    P_aug(n_x_1, n_x_1) = std_yawdd_ * std_yawdd_;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    Xsig_aug.col(0) = x_aug;
    float f = sqrt(lambda_ + n_aug_);
    for (int i = 0; i < n_aug_; i++) {
        //int idx = i < n_x_ ? i : (n_x_-1);
        Xsig_aug.col(i+1)        = x_aug + f * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - f * L.col(i);
    }
}


/**
 * @param measurement_pack meas_package The latest measurement data of
 * either radar or laser.
 */
bool UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
      /********************************https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/daf3dee8-7117-48e8-a27a-fc4769d2b954/concepts/a01da3d5-9c21-409a-b775-9b237987df46*********************************************
       *  Initialization
       ****************************************************************************/
      if (!is_initialized_) {
        // first measurement
        // init timestamp
        time_us_ = measurement_pack.timestamp_;

        // init state with the first measurement
        double x = measurement_pack.raw_measurements_[0];
        double y = measurement_pack.raw_measurements_[1];
        double v = 0.0;
        double yaw = 0.0;
        double yawdd = 0.0;
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
          // Convert radar from polar to cartesian coordinates
          double rho = measurement_pack.raw_measurements_[0];
          double phi = measurement_pack.raw_measurements_[1];
          double rho_dot = measurement_pack.raw_measurements_[2];
          x = rho * cos(phi);
          y = rho * sin(phi);
          yaw = phi;
          double vx = rho_dot * cos(phi);
          double vy = rho_dot * sin(phi);
          v = sqrt(vx*vx + vy*vy);
          if (rho_dot < 0)
              v = -v;
        }
        
        x_ << x, y, v, yaw, yawdd;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return true;
      }

      /*****************************************************************************
       *  Prediction
       ****************************************************************************/
      double dt = (measurement_pack.timestamp_ - time_us_) / 1000000.0;
      time_us_ = measurement_pack.timestamp_;
      if (dt < NEAR_ZERO)
          dt = NEAR_ZERO;
      //create sigma point matrix
      MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_);
      AugmentedSigmaPoints(Xsig_aug);
      SigmaPointPrediction(dt, Xsig_aug);
      PredictMeanAndCovariance();
    
      /*****************************************************************************
       *  Update
       ****************************************************************************/
      // Use the sensor type to perform the update step.
      // Update the state and covariance matrices.
      if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        if (use_radar_)
            UpdateRadar(measurement_pack);
        else
            return false;
      } else {
        // Laser updates
        if (use_laser_)
            UpdateLidar(measurement_pack);
        else
            return false;
      }

      // print the output
      //cout << "x_ = " << x_ << endl;
      //cout << "P_ = " << P_ << endl;
      return true;
}

void UKF::SigmaPointPrediction(double delta_t, MatrixXd& Xsig_aug) {
    // init matrix
    Xsig_pred_.fill(0.0);

    //predict sigma points
    for (int i = 0; i< n_sigma_; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);
        
        //predicted state values
        double px_p, py_p;
        
        //avoid division by zero
        if (fabs(yawd) > NEAR_ZERO) {
            px_p = p_x + v/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }
        
        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;
        
        //add noise
        double nu_dt_2 = 0.5*nu_a*delta_t*delta_t;
        px_p = px_p + nu_dt_2 * cos(yaw);
        py_p = py_p + nu_dt_2 * sin(yaw);
        v_p = v_p + nu_a*delta_t;
        
        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;
        
        //write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }
}

void UKF::PredictMeanAndCovariance() {
    //create vector for predicted state
    VectorXd x = VectorXd(n_x_);
    
    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x_, n_x_);
    
    //predict state mean
    x.fill(0.0);
    for (int i = 0; i < n_sigma_; i++)
        x = x + weights_(i) * Xsig_pred_.col(i).head(n_x_);
    
    //predict state covariance matrix)
    P.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        VectorXd diff = Xsig_pred_.col(i).head(n_x_) - x;
        diff(3) = Tools::normalize_angle(diff(3));
        P = P +  weights_(i) * diff * diff.transpose();
    }
    
    x_ = x;
    P_ = P;
}
    
/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    //create example vector for incoming radar measurement
    VectorXd z = meas_package.raw_measurements_;
    
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_, n_sigma_);
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_);
    
    //transform sigma points into measurement space
    for (int i = 0; i < n_sigma_; i++) {
        double px = Xsig_pred_(0, i);
        double py = Xsig_pred_(1, i);
        double v  = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        double len = sqrt(px*px+py*py);
        Zsig(0, i) = len < NEAR_ZERO ? NEAR_ZERO : len;
        Zsig(1, i) = atan2(py, px);
        Zsig(2, i) = (px*cos(yaw)*v + py*sin(yaw)*v)/Zsig(0, i);
    }

    //calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    
    //calculate measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z_, n_z_);
    S.fill(0.0);

    VectorXd diff = VectorXd(n_z_);
    for (int i = 0; i < n_sigma_; i++) {
        diff = Zsig.col(i) - z_pred;
        diff(1) = Tools::normalize_angle(diff(1));
        S = S + weights_(i) * diff * diff.transpose();
    }
    
    MatrixXd R = MatrixXd(n_z_,n_z_);
    R.fill(0.0);
    R(0,0) = std_radr_ * std_radr_;
    R(1,1) = std_radphi_ * std_radphi_;
    R(2,2) = std_radrd_ * std_radrd_;
    S = S + R;
    
    // cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z_);
    
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        z_diff(1) = Tools::normalize_angle(z_diff(1));
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i).head(n_x_) - x_;
        //angle normalization
        x_diff(3) = Tools::normalize_angle(x_diff(3));
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    //Kalman gain K;
    MatrixXd sinv = S.inverse();
    MatrixXd K = Tc * sinv;
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    //angle normalization
    z_diff(1) = Tools::normalize_angle(z_diff(1));
    
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    
    // NIS
    NIS_radar_ = z_diff.transpose() * sinv * z_diff;
    cout << "NIS_Radar_," << NIS_radar_ << std::endl;
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    //create example vector for incoming laser measurement
    VectorXd z = meas_package.raw_measurements_;
    
    //create matrix for sigma pts in measurement space
    MatrixXd l_sig = MatrixXd(n_l_, n_sigma_);
    
    //mean predicted measurement
    VectorXd l_pred = VectorXd(n_l_);
    
    // transform sigma pts into measurement space
    l_sig.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        float p_x = Xsig_pred_(0, i);
        float p_y = Xsig_pred_(1, i);
        l_sig(0, i) = p_x;
        l_sig(1, i) = p_y;
    }
    
    // calculate mean pred measurement
    l_pred.fill(0.0);
    for (int i = 0; i < n_sigma_; i++)
        l_pred = l_pred + weights_(i) * l_sig.col(i);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_l_, n_l_);
    S.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {
        MatrixXd l_diff = l_sig.col(i) - l_pred;
        S = S + weights_(i) * l_diff * l_diff.transpose();
    }
    
    // residual matrix
    MatrixXd R = MatrixXd(n_l_,n_l_);
    R << std_laspx_ * std_laspx_, 0,
         0,  std_laspy_ * std_laspy_;
    S = S + R;
    
    // cross correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_l_);
    Tc.fill(0.0);
    for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points
        VectorXd l_diff = l_sig.col(i) - l_pred;
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i).head(n_x_) - x_;
        Tc = Tc + weights_(i) * x_diff * l_diff.transpose();
    }
    
    // Kalman Gain K
    MatrixXd sinv = S.inverse();
    MatrixXd K = Tc * sinv;
    
    //residual
    VectorXd z_diff = z - l_pred;
    
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();
    
    // Calculate NIS
    NIS_lidar_ = z_diff.transpose() * sinv * z_diff;
    cout << "NIS_lidar_," << NIS_lidar_ << std::endl;
}




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

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* # of sigma pts
  int n_sigma_;
    
  ///* measurement dimension, radar can measure r, phi, and r_dot
  int n_z_;
  
  ///* measurement dimension, laser can measure x and y
  int n_l_;
    
  ///* Sigma point spreading parameter
  double lambda_;

  ///* NIS
  VectorXd NIS_radar_;
  VectorXd NIS_lidar_;
    
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * Set noise standard deviation longitudinal acceleration and yaw acceleration
   */
  void setSigma(double std_a, double std_yaw)
        { std_a_ = std_a;  std_yawdd_ = std_yaw; }
   
  /**
    * switches for laser and radar
    */
  void disableLaser() { use_laser_ = false; }
  void disableRadar() { use_radar_ = false; }
    
  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   * return true when the measurement is processed
   */
  bool ProcessMeasurement(MeasurementPackage meas_package);

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
    
protected:
    // Calc augmented Sigma Pts
    void AugmentedSigmaPoints(MatrixXd& Xsig_aug);
    // Predict augmented Sigma Pts after delta seconds
    void SigmaPointPrediction(double delta_t, MatrixXd& Xsig_aug);
    // Calc Predict mean and covariance
    void PredictMeanAndCovariance();
    
    void UpdateState(VectorXd* x_out, MatrixXd* P_out);
};

#endif /* UKF_H */

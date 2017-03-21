#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
    
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
    
    // Initialise P (object covariance) as zero matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
    // Initialise F with dt = 0
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    cout << "FusionEKF::FusionEFK() done" << endl;
    
    // Initialise H
    
    
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
      
    // Create process covariance matrix
    ekf_.Q_ = Eigen::MatrixXd(4,4);
    // ekf_.G_ = Eigen::MatrixXd(4,2);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
        float rho = measurement_pack.raw_measurements_[0];
        float phi = measurement_pack.raw_measurements_[1];
        float rhodot = measurement_pack.raw_measurements_[2];
        float px = rho * cos(phi);
        float py = rho * sin(phi);
        float vx = rhodot * cos(phi);
        float vy = rhodot * sin(phi);
        ekf_.x_ << px, py, vx, vy;
        cout << "radar ekf_.x_: " << ekf_.x_ << endl;
        previous_timestamp_ = measurement_pack.timestamp_;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
        ekf_.x_ << measurement_pack.raw_measurements_[0],
        measurement_pack.raw_measurements_[1], 0, 0;
        previous_timestamp_ = measurement_pack.timestamp_;
        cout << "laser ekf_.x_: " << ekf_.x_ << endl;

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    
    cout << "Start predicting" << endl;
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    cout << "dt: " << dt << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
    
    cout << "Modify F matrix" << endl;
    
    //Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    
    cout << "F_: " << ekf_.F_ << endl;
    
    // noise values
    float noise_ax = 9;
    float noise_ay = 9;
    
    // Update G
    /*
    ekf_.G_ << dt_2/2, 0,
               0, dt_2/2,
               dt, 0,
               0, dt;
    */
    
    cout << "Update Q" << endl;
    // Update the process covariance matrix Q
    cout << "Multiply: " << dt_4/4 * noise_ax << endl;
    ekf_.Q_ <<  dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax, 0,
                0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
                dt_3/2 * noise_ax, 0, dt_2 * noise_ax, 0,
                0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;
    cout << "Finished updating Q" << endl;
  ekf_.Predict();
  cout << "Predicted" << endl;
    
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // KalmanFilter::UpdateEKF
      
    // set H_ to Hj when updating with a radar measurement
  } else {
    // Laser updates
      // Laser updates
      // KalmanFilter::Update
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

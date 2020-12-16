//
// Created by atta on 12/15/20.
//

#include "FusionEKF.h"

Helper helper;

FusionEKF::FusionEKF() {
    is_initialized_ = false;
    previous_timestamp_ = 0;

    /**
     * Initialize matrices
     */
    R_laser_ = MatrixXd(2,2);      // laser measurement noise
    R_radar_ = MatrixXd(3,3);      // radar measurement noise
    H_laser_ = MatrixXd(2,4);      // measurement function for laser
    H_jacobian = MatrixXd(3,4);    // measurement function for radar

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // initialize the kalman filter variables
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    // set measurement noises
    noise_ax = 7;
    noise_ay = 7;
}

// Default decostructor
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const Measurement &meas) {
    /**
     * Initialize the EKF matrices
     */
     if (!is_initialized_){
         cout << "Initialization(Radar) is started ..." << endl;
         ekf_.x_ = VectorXd(4);
         if(meas.sensor_type_ == Measurement::SensorType::RADAR){
             float rho = meas.raw_measurements_[0];      // range: radial distance from origin
             float phi = meas.raw_measurements_[1];      // bearing: angle between rho and x axis
             float rho_dot = meas.raw_measurements_[2];  // radial velocity: change of rho

             ekf_.x_ << rho * cos(phi), rho * sin(phi), rho_dot * cos(phi), rho_dot * sin(phi);
         }else if(meas.sensor_type_ == Measurement::SensorType::LIDAR){
             cout << "Initialization(Lidar) is started ..." << endl;
             ekf_.x_ << meas.raw_measurements_[0], meas.raw_measurements_[1], 0.0, 0.0; // x, y, vx, vy
         }else if(meas.sensor_type_ == Measurement::SensorType::IMU){
             cout << "Initialization(IMU) is started ..." << endl;
             ekf_.x_ << 0.0, 0.0, 0.0, 0.0; // x, y, vx, vy
         }else if(meas.sensor_type_ == Measurement::SensorType::GPS){
             cout << "Initialization(GPS) is started ..." << endl;
             ekf_.x_ << 0.0, 0.0, 0.0, 0.0; // x, y, vx, vy
         }else{
             cerr << "Unknown initialization instant!";
         }
         previous_timestamp_ = meas.timestamp_;

         is_initialized_ = true;
     }

    /**
    * Prediction step: Update the state transition
    * matrix F according to the new elapsed time.
    * Update the process noise covariance matrix.
    * Use noise_ax and noise_ay for your Q matrix.
    */
    //Calculate time step
    float dt = (meas.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = meas.timestamp_;

    float dt_2 = pow(dt, 2);
    float dt_3 = pow(dt, 3);
    float dt_4 = pow(dt, 4);

    // Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4/4*noise_ax,   0,                dt_3/2*noise_ax,  0,
                0,                 dt_4/4*noise_ay,  0,                dt_3/2*noise_ay,
                dt_3/2*noise_ax,   0,                dt_2*noise_ax,    0,
                0,                 dt_3/2*noise_ay,  0,                dt_2*noise_ay;

    ekf_.Predict();

    /**
    * Correction step.
    */
    if (meas.sensor_type_ == Measurement::SensorType::RADAR) {
        // RADAR correction
        H_jacobian = helper.CalculateJacobian(ekf_.x_);
        ekf_.H_ = H_jacobian;
        ekf_.R_ = R_radar_;
        ekf_.CorrectEKF(meas.raw_measurements_);
    }else if(meas.sensor_type_ == Measurement::SensorType::LIDAR){
        // LIDAR correction
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Correct(meas.raw_measurements_);
    }else if(meas.sensor_type_ == Measurement::SensorType::IMU) {
        /**
         * TODO:
         * IMU correction
         */
    }else if(meas.sensor_type_ == Measurement::SensorType::GPS){
        /**
         * TODO:
         * GPS correction
         */
    } else{
        cerr << "The sensor type for correction step is unknown" << endl;
    }
}


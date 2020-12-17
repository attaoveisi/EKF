
#ifndef EKF_FUSIONEKF_H
#define EKF_FUSIONEKF_H

#include "measurement.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "kalman_filter.h"
#include "helper.h"

class FusionEKF {
public:
    /**
     * Default constructor
     */
    FusionEKF();

    /**
     * Default deconstructor
     */
     virtual ~FusionEKF();

     /**
      * EKF step
      * @param meas
      */
     void ProcessMeasurement(const Measurement &meas);

     /**
      * Create a KalmanFilter-object
      */
     KalmanFilter ekf_;

private:
    /**
     * The initialization is done once and checked
     */
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    /**
     * Helper function to calculate the Jacobian and the RMSE
     */
    Helper helper;
    MatrixXd R_laser_;      // laser measurement noise
    MatrixXd R_radar_;      // radar measurement noise
    MatrixXd H_laser_;      // measurement function for laser
    MatrixXd H_jacobian;    // measurement function for radar

    float noise_ax;
    float noise_ay;
};


#endif //EKF_FUSIONEKF_H

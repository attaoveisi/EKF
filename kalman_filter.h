/**
 * @file kalman_filer.h
 * @author Atta Oveisi (atta.oveisi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-12-15
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <math.h>
#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;

class KalmanFilter{
public:

    /**
     * State vector
     */
     VectorXd x_;

    /**
    * State covariance
    */
    MatrixXd P_;

    /**
    * State transition matrix
    */
    MatrixXd F_;

    /**
    * Process covariance matrx
    */
    MatrixXd Q_;

    /**
    * Measurement matrix
    */
    MatrixXd H_;

    /**
    * Measurement covariance matrix
    */
    MatrixXd R_

    /**
    * Default constructor
    */
    KalmanFilter();

    /**
    * Default deconstructor
    */
    virtual ~KalmanFilter();

    /**
    * Initialize the Kalmna Filter
    * @param x_in initial states
    * @param P_in initial State covariance
    * @param F_in initial State transition matrix
    * @param H_in initial Measurement matrix
    * @param R_in initial Measurement covariance matrix
    * @param Q_in initial Process covariance matrx
    */
    void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
              MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in);

    /**
    * This step Predicts the state and the state covariance
    * using the process model
    * @param delta_T Time between k and k+1 in s
    */
    void Predict();

    /**
     * convert cartesian to polar coordinates
     * @param x_state is the states in Cartesian coordinates (x, y, vx, vy)
     * @return states in polar coordinates (rho, phi, rho_dot)
     */
    VectorXd CartesianToPolar(const VectorXd &x_state);

    /**
     * Correction step (measurement) for updating the Kalman Filter step
     * @param z The measurement at step k+1
     */
    void Correct(const VectorXd &z);

    /**
     * Correction step (measurement) for updating the Extended Kalman Filter step
     * @param z The measurement at step k+1
     */
    void CorrectEKF(const VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */

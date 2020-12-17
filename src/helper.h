/**
 * @file help.h
 * @author Atta Oveisi (atta.oveisi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-12-15
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef EKF_HELPER_H
#define EKF_HELPER_H

#include <vector>
#include "Eigen/Dense"
#include <iostream>

using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::cerr;

class Helper{
public:
    /**
    * Default constructor.
    */
    Helper();

    /**
    * Default destructor.
    */
    virtual ~Helper();

    /**
    * A helper method to calculate RMSE.
    */
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
    * A helper method to calculate Jacobians.
    */
    MatrixXd CalculateJacobian(const VectorXd& x_state);
};

#endif //EKF_HELPER_H

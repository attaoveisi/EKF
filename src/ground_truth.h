/**
 * @file ground_truth.h
 * @author Atta Oveisi (atta.oveisi@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-12-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef EKF_GROUND_TRUTH_H
#define EKF_GROUND_TRUTH_H

#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;

class GroundTruth{
public:
    long long timestamp_;

    enum SensorType{
        LIDAR,
        RADAR,
        IMU,
        GPS
    }sensor_type_;

    VectorXd gt_values_;

private:

    bool ground_truth_available_{false};

};
#endif //EKF_GROUND_TRUTH_H

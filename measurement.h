/**
 * @file measurement.h
 * @author Atta Oveisi (atta.oveisi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-12-15
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef EKF_MEASUREMENT_H
#define EKF_MEASUREMENT_H

#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;

class Measurement{
public:
    long long timestamp_;

    enum SensorType{
        LIDAR,
        RADAR,
        IMU,
        GPS
    }sensor_type_;

    VectorXd raw_measurements_;

private:

    bool measurement_available_{false};

};

#endif //EKF_MEASUREMENT_H

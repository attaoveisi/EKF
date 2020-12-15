//
// Created by atta on 12/15/20.
//

#ifndef EKF_GROUND_TRUTH_H
#define EKF_GROUND_TRUTH_H

#include "Eigen/Dense"

using Eigen::VectorXd;

class GroundTruth{
public:
    long long timestamp_;

    enum SensorType{
        LASER,
        RADAR
    }sensor_type_;

    VectorXd gt_values_;

private:

    bool groundTruthAvailable = False;

};
#endif //EKF_GROUND_TRUTH_H

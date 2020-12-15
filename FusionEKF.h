
#ifndef EKF_FUSIONEKF_H
#define EKF_FUSIONEKF_H

#include "measurement.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

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


};


#endif //EKF_FUSIONEKF_H

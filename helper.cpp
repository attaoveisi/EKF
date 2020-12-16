//
// Created by atta on 12/16/20.
//
#include "helper.h"

// Default constructor
helper::Helper(){}

// Default deconstructor
helper::~Helper(){}

/**
* RMSE method
*/
VectorXd Helper::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);

    //Initialization
    rmse << 0,0,0,0;

    /**
     * check the validity of the following inputs (size match)
     */
    if(estimations.size() != ground_truth.size() || estimations.size() == 0){
        cerr << "The ground truth data does not match the size of measurement" << endl;
    }

    /**
     * Accumulate squared observation error
     */
    for(auto i:estimations.size()){

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Helper::CalculateJacobian(const VectorXd& x_state) {
    /**
     * Calculate a Jacobian here.
     */
    MatrixXd Hj(3,4);
    Hj << 0,0,0,0,
            0,0,0,0,
            0,0,0,0;

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    /**
     * ill-conditioned Jacobian
     */
    if(fabs(c1) < 0.0001){
        cout << "Warning: Function CalculateJacobian() has Error: Division by Zero. Reassigning rho = 0.0005" << endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px/c2),                (py/c2),                0,      0,
            -(py/c1),               (px/c1),                0,      0,
            py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3,  px/c2,  py/c2;

    return Hj;

}
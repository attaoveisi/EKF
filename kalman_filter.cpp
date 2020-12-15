#include "kalman_filter.h"

// Default constructor
KalmanFilter::KalmanFilter() {}

// Default deconstructor
KalmanFilter::~KalmanFilter() {}

//Init function definition
void KalmanFilter::Init(VectorXd &x_init, MatrixXd &P_init, MatrixXd &F_init, MatrixXd &H_init, MatrixXd &R_init,
                        MatrixXd &Q_init) {
    x_ = x_init;
    P_ = P_init;
    F_ = F_init;
    H_ = H_init;
    R_ = R_init;
    Q_ = Q_init;
}

// Prediction step
void KalmanFilter::Predict(){
    /**
    * predict the state
    */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

/**
 * pdate the state by using Kalman Filter equations
 * @param z the time in sec between steps k & k+1
 */
void KalmanFilter::Update(const VectorXd &z){

    VectorXd z_pred = H_ * x_;

    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

VectorXd KalmanFilter::CartesianToPolar(const VectorXd &x_state){
    float px, py, vx, vy;
    px = x_state[0];
    py = x_state[1];
    vx = x_state[2];
    vy = x_state[3];

    float rho, phi, rho_dot;
    rho = sqrt(px*px + py*py);
    phi = atan2(py, px);  // returns values between -pi and pi

    // if rho is very small, set it to 0.0001 to avoid division by 0 in computing rho_dot
    if(rho < 0.000001){
        rho = 0.000001;
        cout << "The transformation from cartesian to polar is ill-conditioned"
    }

    rho_dot = (px * vx + py * vy) / rho;

    VectorXd z_pred = VectorXd(3);
    z_pred << rho, phi, rho_dot;

    return z_pred;
}
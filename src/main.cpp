/**
 * @file main.cpp
 * @author Atta Oveisi (atta.oveisi@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-12-17
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <fstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth.h"
#include "measurement.h"
#include <matplot/matplot.h>

using std::cerr;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::string;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::cout;

/**
 * The function to check if the user has entered correct number of arguments
 *
 * @param argc
 * @param argv
 */
void check_arguments(int argc, char* argv[]){
    string usage_syntax = "Syntax: ";
    usage_syntax += argv[0];
    usage_syntax += " path/to/input.txt output.txt";

    bool has_valid_args{false};

    switch (argc) {
        case 1:
            cerr << usage_syntax << endl;
        case 2:
            cerr << "Missing output file.\n" << usage_syntax << endl;
        case 3:
            has_valid_args = true;
        case 4 ... INT_MAX:
            cerr << "Too many arguments. \n" << usage_syntax << endl;
        default:
            cout << "Ckecking the validity of syntax ..." << endl;
    }

    if (!has_valid_args){
        exit(EXIT_FAILURE);
    }
}

/**
 * To check if the input/output directories are available
 * @param in_file
 * @param out_file
 * @param in_name
 * @param out_name
 */
void check_files(ifstream &in_file, ofstream &out_file, string& in_name,  string& out_name){

    if(!in_file.is_open()){
        cerr << "Cannot open the input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if(!out_file.is_open()){
        cerr << "Cannot open the output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }

}

/**
 * main driver function
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char* argv[]){
    // check the user arguments
    check_arguments(argc, argv);

    // check the input files
    string in_file_name_ = argv[1];
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    // check the output files
    string out_file_name_ = argv[2];
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    check_files(in_file_, out_file_,in_file_name_, out_file_name_);

    /**
     * Initialize the measurement and ground-truth vectors
     */
    vector<Measurement> measurement_vector;
    vector<GroundTruth> ground_truth_vector;

    /**
     * define line-by-line reading of the data
     */
    string current_line;
    uint64_t n_line{0};

    /**
     * run the loop till all the measurements are used
     */
    while(getline(in_file_, current_line)){

        //Iterate over the n_line
        n_line += 1;

        // For each line, i.e., measurements, define sensor_type,
        // measurement and ground_truth
        string sensor_type;
        Measurement meas;
        GroundTruth gt;
        long long timestamp;

        // Object class of istringstream
        std::istringstream iss(current_line);
        // stream the first element in
        iss >> sensor_type;

        if (sensor_type == "L"){
            // This is a Lidar measurement
            /**
             * Read the measurement from Laser at current time-stamp
             */
             meas.sensor_type_ = Measurement::SensorType::LIDAR;
             meas.raw_measurements_ = VectorXd(2);
             float x,y;

             // extract the Lidar position information
             iss >> x;
             iss >> y;
             meas.raw_measurements_[0] = x;
             meas.raw_measurements_[1] = y;

             //extract the time stamp
             iss >> timestamp;
             meas.timestamp_ = timestamp;

             // save in the measurement vector
             measurement_vector.push_back(meas);
        }else if(sensor_type == "R"){
            // This is a Radar data
            /**
             * Read the measurement from Radar
             */
            meas.sensor_type_ = Measurement::SensorType::RADAR;
            meas.raw_measurements_ = VectorXd(3);
            float ro, phi, ro_dot;

            // extract the Radar distance, and velocity info
            iss >> ro; // Distance to object: RANGE
            iss >> phi; // Angle to the object: BEARING
            iss >> ro_dot; // RADIAL Velocity
            meas.raw_measurements_[0] = ro;
            meas.raw_measurements_[1] = phi;
            meas.raw_measurements_[2] = ro_dot;

            //extract the time stamp
            iss >> timestamp;
            meas.timestamp_ = timestamp;
            measurement_vector.push_back(meas);

        }else if(sensor_type =="I"){
            // This is a IMU data
            /**
             * Read the measurement from IMU
             */
             cout << "IMU is not yet implemented!" << endl;
        }else if(sensor_type =="G"){
            // This is a GPS data
            /**
             * Read the measurement from GPS
             */
            cout << "GPS is not yet implemented!" << endl;
        }else{
            cerr << "There is a line with line number :(" << n_line << ")measurement data without known sensor type (L, R, I, G)" << endl;
        }

        /**
         * Read the ground truth for comparison
         */
         float x_gt, y_gt, vx_gt, vy_gt;
         iss >> x_gt;
         iss >> y_gt;
         iss >> vx_gt;
         iss >> vy_gt;
         gt.gt_values_ = VectorXd(4);
         gt.gt_values_[0] = x_gt;
         gt.gt_values_[1] = y_gt;
         gt.gt_values_[2] = vx_gt;
         gt.gt_values_[3] = vy_gt;
         ground_truth_vector.push_back(gt);
    }

    /**
    * Create a fusion object
    */
    FusionEKF fusionEKF;

    /**
    * For RMSE calculation
    */
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    /**
     * For plotting the results
     */
    vector<float> px_estimated;
    vector<float> px_ground_truth;
    vector<float> py_estimated;
    vector<float> py_ground_truth;
    vector<float> vx_estimated;
    vector<float> vx_ground_truth;
    vector<float> vy_estimated;
    vector<float> vy_ground_truth;

    /**
    * Apply EKF-based fusion:
    * start filtering from the second frame as
    * the speed is unknown in the first
    */
    for(auto k = 0; k < measurement_vector.size(); ++k){
        fusionEKF.ProcessMeasurement(measurement_vector[k]);

        /**
        * output the estimation
        */
        out_file_ << fusionEKF.ekf_.x_(0) << "\t";
        out_file_ << fusionEKF.ekf_.x_(1) << "\t";
        out_file_ << fusionEKF.ekf_.x_(2) << "\t";
        out_file_ << fusionEKF.ekf_.x_(3) << "\t";

        /**
         * output the estimation for plotting
         */
        px_estimated.push_back(fusionEKF.ekf_.x_(0));
        px_ground_truth.push_back(ground_truth_vector[k].gt_values_(0));
        py_estimated.push_back(fusionEKF.ekf_.x_(1));
        py_ground_truth.push_back(ground_truth_vector[k].gt_values_(1));
        vx_estimated.push_back(fusionEKF.ekf_.x_(2));
        vx_ground_truth.push_back(ground_truth_vector[k].gt_values_(2));
        vy_estimated.push_back(fusionEKF.ekf_.x_(3));
        vy_ground_truth.push_back(ground_truth_vector[k].gt_values_(3));

        /**
         * output the measurements
         */
        if(measurement_vector[k].sensor_type_ == Measurement::SensorType::LIDAR) {
            // output the estimation
            out_file_ << measurement_vector[k].raw_measurements_(0) << "\t";
            out_file_ << measurement_vector[k].raw_measurements_(1) << "\t";
        }else if(measurement_vector[k].sensor_type_ == Measurement::SensorType::RADAR) {
            // output the estimation in the cartesian coordinates
            float ro = measurement_vector[k].raw_measurements_(0);
            float phi = measurement_vector[k].raw_measurements_(1);
            out_file_ << ro * cos(phi) << "\t"; // p1_meas
            out_file_ << ro * sin(phi) << "\t"; // ps_meas
        }else if(measurement_vector[k].sensor_type_ == Measurement::SensorType::IMU) {
            /**
             * TODO: Implement IMU output
             */
             cout << "IMU is not yet integrated!" << endl;
        }else if(measurement_vector[k].sensor_type_ == Measurement::SensorType::GPS) {
            /**
             * TODO: Implement GPS output
             */
            cout << "GPS is not yet integrated!" << endl;
        } else{
            cerr << "Unknown sensor type for saving in output files!" << endl;
        }

        /**
         * output the ground truth vector
         */
        out_file_ << ground_truth_vector[k].gt_values_(0) << "\t";
        out_file_ << ground_truth_vector[k].gt_values_(1) << "\t";
        out_file_ << ground_truth_vector[k].gt_values_(2) << "\t";
        out_file_ << ground_truth_vector[k].gt_values_(3) << "\n";

        estimations.push_back(fusionEKF.ekf_.x_);
        ground_truth.push_back(ground_truth_vector[k].gt_values_);
    }

    /**
     * Calculate the RMSE of the estimation
     */
    Helper helper;
    vector<float> px_error(measurement_vector.size(), 0);
    vector<float> py_error(measurement_vector.size(), 0);
    cout << "RMSE is calculated as:" << endl << helper.CalculateRMSE(estimations, ground_truth, px_error, py_error ) << endl;

    /**
     * Close the files
     */
    if (out_file_.is_open()) {
        out_file_.close();
    }
    if (in_file_.is_open()) {
        in_file_.close();
    }

    /**
     * Open a figure1
     */
    matplot::figure(1);
    /**
     * Assign subplots
     */
    matplot::plot(px_estimated, py_estimated, "-o");
    matplot::hold(matplot::on);
    matplot::plot(px_ground_truth, py_ground_truth, "*r");
    matplot::title("Comparison of the estimated position with ground truth.");
    matplot::xlabel( "position x (m)");
    matplot::ylabel("position y (m)");
    matplot::show();
    matplot::save("../filter_output/Tracking.jpg");



    /**
     * Open a figure2
     */
    matplot::figure(2);
    /**
     * Assign subplots
     */
    matplot::tiledlayout(1, 2);
    auto ax1 = matplot::nexttile();
    matplot::plot(ax1, vy_estimated, "-k");
    matplot::hold(matplot::on);
    matplot::plot(ax1, vy_ground_truth, "--r");
    matplot::title(ax1, "Comparison of the estimated lateral velocity with ground truth.");
    matplot::xlabel(ax1, "index");
    matplot::ylabel(ax1, "position v_{y} (m/s)");

    auto ax2 = matplot::nexttile();
    matplot::plot(ax2, vx_estimated, "-k");
    matplot::hold(matplot::on);
    matplot::plot(ax2, vx_ground_truth, "--r");
    matplot::title(ax2, "Comparison of the estimated longitudinal velocity with ground truth.");
    matplot::xlabel(ax2, "index");
    matplot::ylabel(ax2, "position v_{x} (m/s)");
    matplot::show();
    matplot::save("../filter_output/Velocity.jpg");

    /**
     * Open a figure2
     */
    matplot::figure(3);
    /**
     * Assign subplots
     */
    matplot::tiledlayout(1, 2);
    auto ax3 = matplot::nexttile();
    auto h_x = matplot::hist(px_error);
    matplot::xlabel(ax3, "longitudinal error (m)");
    matplot::ylabel(ax3, "histogram");

    auto ax4 = matplot::nexttile();
    auto h_y = matplot::hist(py_error);
    matplot::xlabel(ax4, "lateral error (m)");
    matplot::ylabel(ax4, "histogram");
    matplot::show();
    matplot::save("../filter_output/ErrorDistribution.jpg");

    return 0;
}

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
    cout << "RMSE is calculated as:" << endl << helper.CalculateRMSE(estimations, ground_truth) << endl;

    /**
     * Close the files
     */
    if (out_file_.is_open()) {
        out_file_.close();
    }
    if (in_file_.is_open()) {
        in_file_.close();
    }
    vector<double> x_test = {1,2,3,4,5};
    matplot::plot(x_test);

    return 0;
}
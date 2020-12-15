//
// Created by atta on 12/15/20.
//
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"

#include "ground_truth.h"
#include "measurement.h"

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
            cerr << usage_syntax << endl;
        case 3:
            has_valid_args = true;
        default:
            cerr << "Too many arguments. \n" << usage_syntax << endl;
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
    vector<Measurement> measurement_list;
    vector<GroundTruth> ground_truth_list;

    /**
     * define line-by-line reading of the data
     */
    string current_line;

    /**
     * run the loop till all the measurements are used
     */
    while(getline(in_file_, current_line)){

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

        if (sensor_type.compare("L") == 0){
            // This is a Laser measurement
            /**
             * read the measurement from Laser at current time-stamp
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

        }

        cout << 'The sensor type is: '


    }




}

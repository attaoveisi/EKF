cmake_minimum_required(VERSION 3.17)
project(EKF)

set(CMAKE_CXX_STANDARD 11)

add_library(EKF ground_truth.h main.cpp measurement.h FusionEKF.cpp FusionEKF.h kalman_filter.cpp kalman_filter.h)
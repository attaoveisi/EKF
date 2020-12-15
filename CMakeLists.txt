cmake_minimum_required(VERSION 3.17)
project(EKF)

set(CMAKE_CXX_STANDARD 11)

add_library(EKF library.cpp library.h ground_truth.h main.cpp measurement.h)
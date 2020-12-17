cmake_minimum_required(VERSION 3.5)
project(EKF VERSION 1.0.2)

set(CMAKE_CXX_STANDARD 17)

find_package(Matplot++)

set(source_files src/main.cpp
    src/FusionEKF.cpp
    src/kalman_filter.cpp
    src/helper.cpp)

add_executable(EKF ${source_files})

target_link_libraries(EKF PUBLIC Matplot++::matplot)
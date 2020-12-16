cmake_minimum_required(VERSION 3.17)
project(EKF)

set(CMAKE_CXX_STANDARD 17)

#add_subdirectory(matplotplusplus)
#find_package(Matplot++)


#if (Matplot++_FOUND)
#    add_executable(myprogram main.cpp)
#    target_link_libraries(myprogram PRIVATE Matplot++::matplot)
#endif ()

add_library(EKF ground_truth.h main.cpp measurement.h FusionEKF.cpp FusionEKF.h kalman_filter.cpp kalman_filter.h helper.cpp helper.h)
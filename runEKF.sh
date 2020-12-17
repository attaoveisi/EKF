#!/bin/sh
echo "Running the extended Kalman Filter for fusing the sensor data ..."
echo "Building the package ..."
cd ./build
cmake ..
make
echo "Build successful.! Running ..."
./EKF ../logs/sample-laser-radar-measurement-data-1.txt ../filter_output/output.txt
cd ..
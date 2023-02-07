#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# BSD 3-Clause license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)

project(test_YarpImageOfMonoFloat)

find_package(RobotsIO)
find_package(YARP REQUIRED COMPONENTS os)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(test main.cpp)

target_link_libraries(test PUBLIC RobotsIO::RobotsIO YARP::YARP_init YARP::YARP_os ${OpenCV_LIBS} Eigen3::Eigen)

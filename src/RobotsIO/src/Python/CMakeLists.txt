#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.17)

project(RobotsIOPythonTypes)

# Build shared libs and set PIC on
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# RobotsIO
find_package(RobotsIO REQUIRED)

# Python
find_package(Python3 REQUIRED COMPONENTS Interpreter Development REQUIRED)

# YARP
find_package(YARP REQUIRED)
foreach(_component conf OS sig dev)
  get_property(YARP_${_component}_INCLUDE_DIRS TARGET YARP::YARP_${_component} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  include_directories(${YARP_${_component}_INCLUDE_DIRS})
endforeach()
include_directories(${YARP_BINDINGS})

# SWIG
find_package(SWIG 3.0 REQUIRED)
include(${SWIG_USE_FILE})

# Configure SWIG
set(SWIG_BINDINGS_SOURCE_FILE rgb-filter-data.i)
set_source_files_properties(${SWIG_BINDINGS_SOURCE_FILE} PROPERTIES CPLUSPLUS ON)
set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")
set(CMAKE_SWIG_FLAGS "-py3;-O;-threads")

# Create SWIG library
swig_add_library(RobotsIOPythonTypes
                 LANGUAGE python
                 SOURCES ${SWIG_BINDINGS_SOURCE_FILE})

# Link libraries
swig_link_libraries(RobotsIOPythonTypes Python3::Python YARP::YARP_OS YARP::YARP_sig YARP::YARP_dev RobotsIO::RobotsIO)

# Include directories
target_include_directories(${SWIG_MODULE_RobotsIOPythonTypes_REAL_NAME} SYSTEM PRIVATE ${PYTHON_INCLUDE_PATH})

set_target_properties(${SWIG_MODULE_RobotsIOPythonTypes_REAL_NAME}
                      PROPERTIES OUTPUT_NAME "RobotsIOPythonTypes"
                      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python")

#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.5)

project(RobotsIO
        LANGUAGES CXX
        VERSION 0.0.100)

set(CMAKE_CXX_STANDARD 11)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(WIN32)
    add_definitions(-D_USE_MATH_DEFINES)
    if(MSVC)
        # Since MSVC produces both release and debug
        set(CMAKE_DEBUG_POSTFIX "d")
        # Enable parallel compile and optimized handling of exception filters
        set(CMAKE_CXX_FLAGS "/MP /EHsc")
        set(CMAKE_C_FLAGS "/MP /EHsc")
    endif()
endif()

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

# Option for tests
# option(BUILD_TESTING "Create tests using CMake" OFF)
# if(BUILD_TESTING)
#   message(STATUS "Test enabled")
#   enable_testing()
# endif()

# Enable RPATH
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

# Default build type to Release
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
  endif()

# options
option(USE_YARP "Use YARP" OFF)
option(USE_ICUB "Use ICUB" OFF)
option(USE_SUPERIMPOSE "Use SuperimposeMesh" OFF)

# if ICUB is available use ICUBcontrib helpers for installation
if (USE_ICUB)
  # iCub contrib
  find_package(ICUBcontrib REQUIRED)

  list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
  include(ICUBcontribHelpers)
  include(ICUBcontribOptions)

  icubcontrib_set_default_prefix()
  icubcontrib_add_uninstall_target()
endif()

# Library sources
add_subdirectory(src)

# Install the files necessary to call find_package(RobotsIO) in CMake projects

# Dependencies
set(DEPENDENCIES "Eigen3")
if (USE_YARP)
  set(DEPENDENCIES ${DEPENDENCIES} "YARP COMPONENTS cv dev eigen os sig")
endif()
if (USE_ICUB)
  set(DEPENDENCIES ${DEPENDENCIES} "ICUB")
endif()
if (USE_SUPERIMPOSE)
  set(DEPENDENCIES ${DEPENDENCIES} "SuperimposeMesh")
endif()

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY ExactVersion
                            EXPORT ${PROJECT_NAME}
                            NO_SET_AND_CHECK_MACRO
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            DEPENDENCIES ${DEPENDENCIES})

if (NOT USE_ICUB)
  # Add standard uninstall target
  include(AddUninstallTarget)
endif()

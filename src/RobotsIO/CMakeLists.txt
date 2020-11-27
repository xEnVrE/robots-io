#===============================================================================
#
# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

option(USE_JSONCPP "Use jsoncpp (this enables BOPDatasetDetection)" OFF)

set(LIBRARY_TARGET_NAME RobotsIO)

# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

if (USE_JSONCPP)
   # jsoncpp
   find_package(PkgConfig REQUIRED)
   pkg_check_modules(JSONCPP jsoncpp)
endif()

if (USE_SUPERIMPOSE)
    # SuperimposeMesh
    find_package(SuperimposeMesh 0.11.100 REQUIRED)
endif()

if (USE_YARP)
    find_package(YARP REQUIRED COMPONENTS
                 cv
                 dev
                 eigen
                 os
                 sig
    )
endif()

if (USE_ICUB)
    find_package(ICUB REQUIRED)
endif()

# Header files
set(${LIBRARY_TARGET_NAME}_HDR_CAMERA
    include/RobotsIO/Camera/Camera.h
    include/RobotsIO/Camera/CameraDeprojectionMatrix.h
    include/RobotsIO/Camera/CameraParameters.h
    include/RobotsIO/Camera/DatasetCamera.h
    include/RobotsIO/Camera/DatasetParameters.h
)

set(${LIBRARY_TARGET_NAME}_HDR_HAND "")

set(${LIBRARY_TARGET_NAME}_HDR_UTILS
    include/RobotsIO/Utils/Clock.h
    include/RobotsIO/Utils/ClockedComponent.h
    include/RobotsIO/Utils/Data.h
    include/RobotsIO/Utils/DataStream.h
    include/RobotsIO/Utils/DatasetDataStream.h
    include/RobotsIO/Utils/DatasetDataStreamDelayed.h
    include/RobotsIO/Utils/DatasetDetection.h
    include/RobotsIO/Utils/DatasetSpatialVelocity.h
    include/RobotsIO/Utils/DatasetTransform.h
    include/RobotsIO/Utils/DatasetTransformDelayed.h
    include/RobotsIO/Utils/DepthToFile.h
    include/RobotsIO/Utils/Detection.h
    include/RobotsIO/Utils/DetectionYarpPort.h
    include/RobotsIO/Utils/FileToDepth.h
    include/RobotsIO/Utils/FileToEigen.h
    include/RobotsIO/Utils/FloatMatrix.h
    include/RobotsIO/Utils/ImageFileProbe.h
    include/RobotsIO/Utils/Parameters.h
    include/RobotsIO/Utils/ParametersExtractor.h
    include/RobotsIO/Utils/ParametersFiller.h
    include/RobotsIO/Utils/Probe.h
    include/RobotsIO/Utils/ProbeContainer.h
    include/RobotsIO/Utils/SpatialVelocity.h
    include/RobotsIO/Utils/SpatialVelocityBuffer.h
    include/RobotsIO/Utils/Transform.h
    include/RobotsIO/Utils/TransformWithVelocity.h
    include/RobotsIO/Utils/any.h
)


# Source files
set(${LIBRARY_TARGET_NAME}_SRC_CAMERA
    src/Camera/Camera.cpp
    src/Camera/CameraDeprojectionMatrix.cpp
    src/Camera/CameraParameters.cpp
    src/Camera/DatasetCamera.cpp
    src/Camera/DatasetParameters.cpp
)

set(${LIBRARY_TARGET_NAME}_SRC_HAND "")

set(${LIBRARY_TARGET_NAME}_SRC_UTILS
    src/Utils/Clock.cpp
    src/Utils/ClockedComponent.cpp
    src/Utils/DataStream.cpp
    src/Utils/DatasetDataStream.cpp
    src/Utils/DatasetDataStreamDelayed.cpp
    src/Utils/DatasetDetection.cpp
    src/Utils/DatasetSpatialVelocity.cpp
    src/Utils/DatasetTransform.cpp
    src/Utils/DatasetTransformDelayed.cpp
    src/Utils/DepthToFile.cpp
    src/Utils/Detection.cpp
    src/Utils/DetectionYarpPort.cpp
    src/Utils/FileToDepth.cpp
    src/Utils/FileToEigen.cpp
    src/Utils/FloatMatrix.cpp
    src/Utils/ImageFileProbe.cpp
    src/Utils/Parameters.cpp
    src/Utils/ParametersExtractor.cpp
    src/Utils/Probe.cpp
    src/Utils/ProbeContainer.cpp
    src/Utils/SpatialVelocity.cpp
    src/Utils/SpatialVelocityBuffer.cpp
    src/Utils/Transform.cpp
    src/Utils/TransformWithVelocity.cpp
    )

if (USE_JSONCPP)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR_UTILS
         include/RobotsIO/Utils/BOPDatasetDetection.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC_UTILS
         src/Utils/BOPDatasetDetection.cpp
    )
endif()

if (USE_SUPERIMPOSE)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR_CAMERA
         include/RobotsIO/Camera/SegmentationCamera.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC_CAMERA
         src/Camera/SegmentationCamera.cpp
    )
endif()

if (USE_YARP)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR_CAMERA
         include/RobotsIO/Camera/RealsenseCameraYarp.h
         include/RobotsIO/Camera/YarpCamera.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_HDR_UTILS
         include/RobotsIO/Utils/ClockYarp.h
         include/RobotsIO/Utils/FloatMatrixYarpPort.h
         include/RobotsIO/Utils/Parameters2YarpBottle.h
         include/RobotsIO/Utils/ParametersYarpPort.h
         include/RobotsIO/Utils/RGBFilterData.h
         include/RobotsIO/Utils/SpatialVelocityYarpPort.h
         include/RobotsIO/Utils/TransformYarpTransformClient.h
         include/RobotsIO/Utils/TransformWithVelocityYarpPort.h
         include/RobotsIO/Utils/TransformYarpPort.h
         include/RobotsIO/Utils/YarpBottleProbe.hpp
         include/RobotsIO/Utils/YarpBufferedPort.hpp
         include/RobotsIO/Utils/YarpImageOfProbe.hpp
         include/RobotsIO/Utils/YarpRGBFilterDataProbe.h
         include/RobotsIO/Utils/YarpVectorOfProbe.hpp
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC_UTILS
         src/Utils/ClockYarp.cpp
         src/Utils/FloatMatrixYarpPort.cpp
         src/Utils/Parameters2YarpBottle.cpp
         src/Utils/ParametersYarpPort.cpp
         src/Utils/RGBFilterData.cpp
         src/Utils/SpatialVelocityYarpPort.cpp
         src/Utils/TransformYarpTransformClient.cpp
         src/Utils/TransformWithVelocityYarpPort.cpp
         src/Utils/TransformYarpPort.cpp
         src/Utils/YarpBottleProbe.cpp
         src/Utils/YarpRGBFilterDataProbe.cpp
         src/Utils/YarpVectorOfProbe.cpp
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC_CAMERA
         src/Camera/YarpCamera.cpp
         src/Camera/RealsenseCameraYarp.cpp
    )
endif()

if (USE_YARP AND USE_ICUB)
    list(APPEND ${LIBRARY_TARGET_NAME}_HDR_CAMERA
         include/RobotsIO/Camera/iCubCamera.h
         include/RobotsIO/Camera/iCubCameraDepth.h
         include/RobotsIO/Camera/iCubCameraRelative.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_HDR_HAND
         include/RobotsIO/Hand/iCubHand.h
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC_CAMERA
         src/Camera/iCubCamera.cpp
         src/Camera/iCubCameraDepth.cpp
         src/Camera/iCubCameraRelative.cpp
    )

    list(APPEND ${LIBRARY_TARGET_NAME}_SRC_HAND
         src/Hand/iCubHand.cpp
    )

    set(${LIBRARY_TARGET_NAME}_ICUBHAND_CONF config/Hand/icub_hand_configuration.ini.template)

endif()

set(${LIBRARY_TARGET_NAME}_HDR
        ${${LIBRARY_TARGET_NAME}_HDR_CAMERA}
        ${${LIBRARY_TARGET_NAME}_HDR_HAND}
        ${${LIBRARY_TARGET_NAME}_HDR_UTILS}
)

set(${LIBRARY_TARGET_NAME}_SRC
        ${${LIBRARY_TARGET_NAME}_SRC_CAMERA}
        ${${LIBRARY_TARGET_NAME}_SRC_HAND}
        ${${LIBRARY_TARGET_NAME}_SRC_UTILS}
)

# Add library
add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

# Library properties
# set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${${PROJECT_NAME}_VERSION}
#                                                         PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

# Include directories
target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Linker configuration
target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC Eigen3::Eigen ${OpenCV_LIBS})

if (USE_JSONCPP)
   message(${JSONCPP_LIBRARIES})
   target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC ${JSONCPP_LIBRARIES})
endif()

if (USE_YARP)
    target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC
                          YARP::YARP_cv
                          YARP::YARP_dev
                          YARP::YARP_eigen
                          YARP::YARP_init
                          YARP::YARP_os
                          YARP::YARP_sig
    )
endif()

if (USE_ICUB)
    target_link_libraries(${LIBRARY_TARGET_NAME} PRIVATE ICUB::iKin ICUB::learningMachine)
endif()

if (USE_SUPERIMPOSE)
    target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC SI::SuperimposeMesh)
endif()

# Specify installation targets, typology and destination folders.
install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                   COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                   COMPONENT bin
        # PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}"        COMPONENT dev
)

install(FILES         ${${LIBRARY_TARGET_NAME}_HDR_CAMERA}
        DESTINATION  "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/Camera")
install(FILES         ${${LIBRARY_TARGET_NAME}_HDR_HAND}
        DESTINATION  "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/Hand")
install(FILES         ${${LIBRARY_TARGET_NAME}_HDR_UTILS}
        DESTINATION  "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}/Utils")

set_property(GLOBAL APPEND PROPERTY ${PROJECT_NAME}_TARGETS ${LIBRARY_TARGET_NAME})

if (USE_YARP AND USE_ICUB)
    yarp_install(FILES ${${LIBRARY_TARGET_NAME}_ICUBHAND_CONF} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/robots_io_icub_hand)
endif()

if (BUILD_TESTING)
  add_subdirectory(test)
endif()

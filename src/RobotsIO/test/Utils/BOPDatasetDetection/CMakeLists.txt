#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(TEST_TARGET_NAME test_BOPDatasetDetection)

set(${TEST_TARGET_NAME}_HDR
)

set(${TEST_TARGET_NAME}_SRC
    main.cpp
)

add_executable(${TEST_TARGET_NAME} ${${TEST_TARGET_NAME}_HDR} ${${TEST_TARGET_NAME}_SRC})

target_include_directories(${TEST_TARGET_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(${TEST_TARGET_NAME} RobotsIO)

add_test(NAME ${TEST_TARGET_NAME}
         COMMAND ${TEST_TARGET_NAME}
         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

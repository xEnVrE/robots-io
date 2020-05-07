/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_CAMERADEPROJECTIONMATRIX_H
#define ROBOTSIO_CAMERADEPROJECTIONMATRIX_H

#include "RobotsIO/Camera/CameraParameters.h"
#include <RobotsIO/Camera/CameraParameters.h>

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Camera {
        Eigen::MatrixXd deprojection_matrix(const std::size_t& width, const std::size_t& height, const double& fx, const double& fy, const double& cx, const double& cy);

        Eigen::MatrixXd deprojection_matrix(const CameraParameters& parameters);
    }
}

#endif

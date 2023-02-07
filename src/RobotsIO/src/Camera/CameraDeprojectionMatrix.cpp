/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/CameraDeprojectionMatrix.h>

using namespace Eigen;


MatrixXd RobotsIO::Camera::deprojection_matrix(const std::size_t& width, const std::size_t& height, const double& fx, const double& fy, const double& cx, const double& cy)
{
    MatrixXd deprojection_matrix(3, width * height);

    std::size_t i = 0;
    for (std::size_t u = 0; u < width; u++)
    {
        for (std::size_t v = 0; v < height; v++)
        {
            deprojection_matrix(0, i) = (u - cx) / fx;
            deprojection_matrix(1, i) = (v - cy) / fy;
            deprojection_matrix(2, i) = 1.0;

            i++;
        }
    }

    return deprojection_matrix;
}


Eigen::MatrixXd RobotsIO::Camera::deprojection_matrix(const CameraParameters& parameters)
{
    return RobotsIO::Camera::deprojection_matrix(parameters.width(), parameters.height(), parameters.fx(), parameters.fy(), parameters.cx(), parameters.cy());
}

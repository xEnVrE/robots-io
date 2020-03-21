/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/SpatialVelocity.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


Eigen::Vector3d SpatialVelocity::angular_velocity()
{
    return twist().segment<3>(3);
}


Eigen::Vector3d SpatialVelocity::linear_velocity_origin()
{
    return twist().head<3>();
}


Eigen::Vector3d SpatialVelocity::linear_velocity_screw()
{
    double angular_norm = twist().segment<3>(3).norm();

    if (angular_norm > 1e-4)
        return twist().head<3>() + twist().segment<3>(3).cross(twist().segment<3>(3).cross(twist().head<3>())) / std::pow(angular_norm, 2);

    return twist().head<3>();
}


Eigen::Vector3d SpatialVelocity::screw_position()
{
    double angular_norm = twist().segment<3>(3).norm();

    if (angular_norm > 1e-4)
        return twist().segment<3>(3).cross(twist().head<3>()) / std::pow(angular_norm, 2);

    return Vector3d::Zero();
}


bool SpatialVelocity::is_screw_degenerate()
{
    return (twist().segment<3>(3).norm() <= 1e-4);
}

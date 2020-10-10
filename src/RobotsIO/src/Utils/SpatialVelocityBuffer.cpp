/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/SpatialVelocityBuffer.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


SpatialVelocityBuffer::SpatialVelocityBuffer()
{
    twist_.resize(6);
}

SpatialVelocityBuffer::~SpatialVelocityBuffer()
{}


bool SpatialVelocityBuffer::freeze(const bool blocking)
{
    return true;
}


void SpatialVelocityBuffer::set_twist(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity, const double& elapsed_time)
{
    twist_.head<3>() = linear_velocity;
    twist_.segment<3>(3) = angular_velocity;
    elapsed_time_ = elapsed_time;
}


double SpatialVelocityBuffer::elapsed_time()
{
    return elapsed_time_;
}


Eigen::VectorXd SpatialVelocityBuffer::twist()
{
    return twist_;
}

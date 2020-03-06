/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/SpatialVelocityYarpPort.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::eigen;
using namespace yarp::sig;


SpatialVelocityYarpPort::SpatialVelocityYarpPort(const std::string& port_name) :
    YarpVectorOfProbe<double>(port_name)
{}


SpatialVelocityYarpPort:: ~SpatialVelocityYarpPort()
{}


std::tuple<bool, Vector3d, Vector3d, Vector3d, double> SpatialVelocityYarpPort::velocity(const bool& blocking)
{
    /* Data reception .*/
    Vector* velocity_yarp = receive_data(blocking);
    if (velocity_yarp == nullptr)
        return std::make_tuple(false, Vector3d(), Vector3d(), Vector3d(), 0.0);

    /* Elapsed time. */
    double elapsed = 0.0;
    auto now = std::chrono::steady_clock::now();
    if (last_time_initialized_)
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count() / 1000.0;
    last_time_ = now;
    last_time_initialized_ = true;

    /* Vector composition. */
    VectorXd velocity = toEigen(*velocity_yarp);
    Vector3d screw_axis = velocity.head<3>();
    Vector3d screw_direction = velocity.segment<3>(3);
    double scalar_linear_velocity = velocity(6);
    double scalar_angular_velocity = velocity(7);
    Vector3d linear_velocity = scalar_linear_velocity * screw_direction;
    Vector3d angular_velocity = scalar_angular_velocity * screw_direction;

    return std::make_tuple(true, screw_axis, linear_velocity, angular_velocity, elapsed);
}

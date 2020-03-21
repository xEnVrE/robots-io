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


bool SpatialVelocityYarpPort::freeze(const bool blocking)
{
    /* Data reception .*/
    Vector* velocity_yarp = receive_data(blocking);
    if (velocity_yarp == nullptr)
        return false;

    /* Elapsed time. */
    elapsed_time_ = 0.0;
    auto now = std::chrono::steady_clock::now();
    if (last_time_initialized_)
        elapsed_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count() / 1000.0;
    last_time_ = now;
    last_time_initialized_ = true;

    twist_ = toEigen(*velocity_yarp);

    return true;
}

double SpatialVelocityYarpPort::elapsed_time()
{
    return elapsed_time_;
}


VectorXd SpatialVelocityYarpPort::twist()
{
    return twist_;
}

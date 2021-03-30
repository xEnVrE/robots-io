/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/TransformWithVelocityYarpPort.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::eigen;
using namespace yarp::sig;


TransformWithVelocityYarpPort::TransformWithVelocityYarpPort(const std::string& port_name) :
    YarpBufferedPort<yarp::sig::Vector>(port_name)
{}


TransformWithVelocityYarpPort:: ~TransformWithVelocityYarpPort()
{}


Eigen::Transform<double, 3, Affine> TransformWithVelocityYarpPort::transform()
{
    return transform_;
}


Eigen::Vector3d TransformWithVelocityYarpPort::linear_velocity()
{
    return linear_velocity_;
}


Eigen::Vector3d TransformWithVelocityYarpPort::angular_velocity()
{
    return angular_velocity_;
}


bool TransformWithVelocityYarpPort::freeze(const bool blocking)
{
    yarp::sig::Vector* transform_yarp = receive_data(blocking);

    if (transform_yarp == nullptr)
        return false;

    transform_ = Translation<double, 3>(toEigen(*transform_yarp).head<3>());
    AngleAxisd rotation((*transform_yarp)(6), toEigen(*transform_yarp).segment<3>(3));
    transform_.rotate(rotation);

    linear_velocity_ = toEigen(*transform_yarp).segment<3>(7);
    angular_velocity_ = toEigen(*transform_yarp).tail<3>();

    return true;
}

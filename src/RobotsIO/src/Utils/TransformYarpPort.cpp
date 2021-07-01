/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/TransformYarpPort.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::eigen;
using namespace yarp::sig;


TransformYarpPort::TransformYarpPort(const std::string& port_name) :
    YarpBufferedPort<yarp::sig::Vector>(port_name)
{}


TransformYarpPort:: ~TransformYarpPort()
{}


Eigen::Transform<double, 3, Affine> TransformYarpPort::transform()
{
    return transform_;
}


bool TransformYarpPort::freeze(const bool blocking)
{
    yarp::sig::Vector* transform_yarp = receive_data(blocking);

    if (transform_yarp == nullptr)
        return false;

    bool invalid_pose = true;
    for (std::size_t i = 0; i < transform_yarp->size(); i++)
        invalid_pose &= ((*transform_yarp)(i) == 0.0);
    if (invalid_pose)
        return false;

    transform_ = Translation<double, 3>(toEigen(*transform_yarp).head<3>());
    AngleAxisd rotation((*transform_yarp)(6), toEigen(*transform_yarp).segment<3>(3));
    transform_.rotate(rotation);

    return true;
}

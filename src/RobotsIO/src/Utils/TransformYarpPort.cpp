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
    YarpVectorOfProbe<double>(port_name)
{}


TransformYarpPort:: ~TransformYarpPort()
{}


std::pair<bool, Eigen::Transform<double, 3, Affine>> TransformYarpPort::transform(const bool& blocking)
{
    Vector* transform_yarp = receive_data(blocking);

    if (transform_yarp == nullptr)
        return std::make_pair(false, Eigen::Transform<double, 3, Affine>());

    Eigen::Transform<double, 3, Affine> transform;
    transform = Translation<double, 3>(toEigen(*transform_yarp).head<3>());
    AngleAxisd rotation((*transform_yarp)(6), toEigen(*transform_yarp).segment<3>(3));
    transform.rotate(rotation);

    return std::make_pair(true, transform);
}

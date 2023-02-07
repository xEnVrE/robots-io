/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/YarpVectorOfProbe.hpp>

using namespace RobotsIO::Utils;


template <>
yarp::sig::VectorOf<double> RobotsIO::Utils::YarpVectorOfProbe<double, Eigen::VectorXd>::convert_from(const Eigen::VectorXd& data)
{
    yarp::sig::VectorOf<double> tmp(data.size());
    yarp::eigen::toEigen(tmp) = data;

    return tmp;
}


template <>
yarp::sig::VectorOf<double> RobotsIO::Utils::YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>::convert_from(const Eigen::Transform<double, 3, Eigen::Affine>& data)
{
    /* Assume by default transformation to x-y-z-axis-angle. */
    yarp::sig::VectorOf<double> tmp(7);
    yarp::eigen::toEigen(tmp).head<3>() = data.translation();

    Eigen::AngleAxisd axis_angle(data.rotation());
    yarp::eigen::toEigen(tmp).segment<3>(3) = axis_angle.axis();
    yarp::eigen::toEigen(tmp)(6) = axis_angle.angle();

    return tmp;
}


template <>
yarp::sig::VectorOf<double> RobotsIO::Utils::YarpVectorOfProbe<double, RobotsIO::Utils::TransformWithVelocityStorage>::convert_from(const RobotsIO::Utils::TransformWithVelocityStorage& data)
{
    /* Assume by default transformation to x-y-z-axis-angle. */
    yarp::sig::VectorOf<double> tmp(13);
    yarp::eigen::toEigen(tmp).head<3>() = data.transform.translation();

    Eigen::AngleAxisd axis_angle(data.transform.rotation());
    yarp::eigen::toEigen(tmp).segment<3>(3) = axis_angle.axis();
    yarp::eigen::toEigen(tmp)(6) = axis_angle.angle();

    yarp::eigen::toEigen(tmp).segment<3>(7) = data.linear_velocity;
    yarp::eigen::toEigen(tmp).tail<3>() = data.angular_velocity;

    return tmp;
}

template <>
yarp::sig::VectorOf<int> RobotsIO::Utils::YarpVectorOfProbe<int, cv::Rect>::convert_from(const cv::Rect& data)
{
    /* Assume by default a top_left x, top_left right, width, height data. */
    yarp::sig::VectorOf<int> tmp(4);
    tmp(0) = data.x;
    tmp(1) = data.y;
    tmp(2) = data.width;
    tmp(3) = data.height;

    return tmp;
}

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/TransformYarpPort.h>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::cv;
using namespace yarp::eigen;
using namespace yarp::sig;


TransformYarpPort::TransformYarpPort(const std::string& port_prefix, const bool& provide_rgb) :
    YarpBufferedPort<yarp::sig::Vector>(port_prefix + "/transform:i"),
    rgb_out_(port_prefix + "/rgb:o"),
    provide_rgb_(provide_rgb)
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


void TransformYarpPort::set_rgb_image(const cv::Mat& image)
{
    if (!provide_rgb_)
        return;

    cv_rgb_out_ = image.clone();
    yarp_rgb_out_ = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(cv_rgb_out_);

    rgb_out_.send_data(yarp_rgb_out_);
}

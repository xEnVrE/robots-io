/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/FloatMatrixYarpPort.h>

#include <opencv2/core/eigen.hpp>

#include <yarp/cv/Cv.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::cv;
using namespace yarp::sig;


FloatMatrixYarpPort::FloatMatrixYarpPort(const std::string& port_name) :
    YarpBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>>(port_name)
{}

FloatMatrixYarpPort::~FloatMatrixYarpPort()
{}


bool FloatMatrixYarpPort::freeze(const bool blocking)
{
    ImageOf<PixelFloat>* image_float_yarp = receive_data(blocking);

    if (image_float_yarp == nullptr)
        return false;

    cv::Mat image_float_cv = toCvMat(*image_float_yarp);

    cv::cv2eigen(image_float_cv, matrix_);

    return true;
}


MatrixXf FloatMatrixYarpPort::matrix()
{
    return matrix_;
}

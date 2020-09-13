/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/YarpRGBFilterDataProbe.h>

#include <Eigen/Core>

#include <yarp/eigen/Eigen.h>
#include <yarp/cv/Cv.h>
#include <yarp/sig/Image.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::cv;
using namespace yarp::sig;
using namespace yarp::eigen;


RobotsIO::Utils::YarpRGBFilterDataProbe::YarpRGBFilterDataProbe(const std::string& port_name) :
    YarpBufferedPort<RGBFilterData>(port_name)
{}


RobotsIO::Utils::YarpRGBFilterDataProbe::~YarpRGBFilterDataProbe()
{}

#include <iostream>
void RobotsIO::Utils::YarpRGBFilterDataProbe::on_new_data()
{
    std::tie(image_, matrix_) = RobotsIO::Utils::any_cast<std::pair<cv::Mat, MatrixXd>>(get_data());

    RGBFilterData& data = port_.prepare();

    data.predicted_pose.resize(matrix_.rows(), matrix_.cols());
    toEigen(data.predicted_pose) = matrix_;

    data.measured_image = fromCvMat<yarp::sig::PixelRgb>(image_);

    port_.write();
}

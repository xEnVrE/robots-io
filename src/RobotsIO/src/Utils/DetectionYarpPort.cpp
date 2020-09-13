/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/DetectionYarpPort.h>

#include <yarp/eigen/Eigen.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::eigen;
using namespace yarp::sig;


DetectionYarpPort::DetectionYarpPort(const std::string& port_name) :
    YarpBufferedPort<yarp::sig::VectorOf<int>>(port_name)
{}


DetectionYarpPort:: ~DetectionYarpPort()
{}


cv::Rect DetectionYarpPort::detection()
{
    return detection_;
}


bool DetectionYarpPort::freeze(const bool blocking)
{
    VectorOf<int>* detection_data = receive_data(blocking);

    if (detection_data == nullptr)
        return false;

    detection_ = cv::Rect((*detection_data)(0), (*detection_data)(1), (*detection_data)(2), (*detection_data)(3));

    return true;
}

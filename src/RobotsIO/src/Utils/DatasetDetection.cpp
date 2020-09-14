/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/DatasetDetection.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


DatasetDetection::DatasetDetection(const std::string& file_path) :
    DatasetDataStream(file_path, 0, 0, 4)
{}


DatasetDetection::~DatasetDetection()
{}


bool DatasetDetection::freeze(const bool blocking)
{
    if (!DatasetDataStream::freeze())
        return false;

    VectorXd bounding_box_data = data();

    detection_ = cv::Rect(bounding_box_data(0), bounding_box_data(1), bounding_box_data(2), bounding_box_data(3));

    return true;
}


cv::Rect DatasetDetection::detection()
{
    return detection_;
}

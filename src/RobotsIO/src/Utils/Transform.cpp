/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Transform.h>

using namespace RobotsIO::Utils;


Transform::~Transform()
{}


int Transform::get_frames_between_iterations() const
{
    /* 1 indicates that the transform stream is, by default, available at all frames. */
    return 1;
}


void Transform::set_rgb_image(const cv::Mat& image)
{
    /* By default, the input image is not used. */
}


void Transform::set_depth_segmentation_image(const Eigen::MatrixXf& depth, const cv::Mat& segmentation)
{
    /* By default, the input images are not used. */
}


bool Transform::transform_received()
{
    /* By default, it returns true. */

    return true;
}

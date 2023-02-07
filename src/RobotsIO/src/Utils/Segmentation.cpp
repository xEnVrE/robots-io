/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Segmentation.h>

using namespace RobotsIO::Utils;


Segmentation::~Segmentation()
{}


bool Segmentation::reset()
{
    return true;
}


bool Segmentation::step_frame()
{
    return true;
}


void Segmentation::reset_data_loading_time()
{
}


double Segmentation::get_data_loading_time() const
{
    return 0.0;
}


int Segmentation::get_frames_between_iterations() const
{
    return 1;
}


std::pair<bool, cv::Mat> Segmentation::latest_segmentation()
{
    return std::make_pair(false, cv::Mat());
}


double Segmentation::get_time_stamp()
{
    return -1;
}


void Segmentation::set_rgb_image(const cv::Mat& image, const double& timestamp)
{
    /* By default, the input image is not used. */
}

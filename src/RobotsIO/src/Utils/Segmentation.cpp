/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
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


void set_rgb_image(const cv::Mat& image)
{
    /* By default, the input image is not used. */
}

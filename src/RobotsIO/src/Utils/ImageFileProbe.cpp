/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/ImageFileProbe.h>

#include <opencv2/opencv.hpp>

using namespace RobotsIO::Utils;


ImageFileProbe::ImageFileProbe(const std::string& output_path, const std::string& prefix, const std::string& output_format) :
    output_prefix_(output_path),
    output_format_(output_format)
{
    if (output_prefix_.back() != '/')
        output_prefix_ += '/';

    if (!prefix.empty())
        output_prefix_ += (prefix + "_");
}


ImageFileProbe::~ImageFileProbe()
{}


void ImageFileProbe::on_new_data()
{
    data_cv_ = RobotsIO::Utils::any_cast<cv::Mat>(get_data());

    cv::imwrite(output_prefix_ + std::to_string(frame_counter_) + "." + output_format_, data_cv_);

    frame_counter_++;
}

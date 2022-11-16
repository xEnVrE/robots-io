/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/SegmentationYarpPort.h>

#include <yarp/cv/Cv.h>

using namespace RobotsIO::Utils;
using namespace yarp::cv;
using namespace yarp::sig;


SegmentationYarpPort::SegmentationYarpPort(const std::string& port_prefix, const bool& provide_rgb) :
    segmentation_in_(port_prefix + "/segmentation:i"),
    rgb_out_(port_prefix + "/rgb:o"),
    provide_rgb_(provide_rgb)
{}


SegmentationYarpPort::~SegmentationYarpPort()
{}


bool SegmentationYarpPort::reset()
{
    segmentation_in_.flush();

    return true;
}


bool SegmentationYarpPort::is_stepping_required() const
{
    return false;
}


int SegmentationYarpPort::get_frames_between_iterations() const
{
    return -1;
}


std::pair<bool, cv::Mat> SegmentationYarpPort::segmentation(const bool& blocking)
{
    ImageOf<PixelMono>* yarp_mask = segmentation_in_.receive_data(blocking);

    if (yarp_mask == nullptr)
    {
        cv_mask_in_ = cv::Mat();
        return std::make_pair(false, cv::Mat());
    }

    yarp_mask_in_.copy(*yarp_mask);
    cv_mask_in_ = toCvMat(yarp_mask_in_);
    time_stamp_mask_in_ = segmentation_in_.time_stamp();

    return std::make_pair(true, cv_mask_in_);
}


std::pair<bool, cv::Mat> SegmentationYarpPort::latest_segmentation()
{
    if (cv_mask_in_.empty())
        return std::make_pair(false, cv::Mat());

    return std::make_pair(true, cv_mask_in_);
}


double SegmentationYarpPort::get_time_stamp()
{
    return time_stamp_mask_in_;
}


void SegmentationYarpPort::set_rgb_image(const cv::Mat& image, const double& timestamp)
{
    if (!provide_rgb_)
        return;

    cv_rgb_out_ = image.clone();
    yarp_rgb_out_ = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(cv_rgb_out_);

    rgb_out_.set_time_stamp(timestamp);
    rgb_out_.send_data(yarp_rgb_out_);
}

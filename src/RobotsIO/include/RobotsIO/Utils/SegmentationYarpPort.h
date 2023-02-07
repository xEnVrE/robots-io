/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_SEGMENTATIONYARPPORT_H
#define ROBOTSIO_SEGMENTATIONYARPPORT_H

#include <RobotsIO/Utils/Segmentation.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>

#include <yarp/sig/Image.h>

namespace RobotsIO {
    namespace Utils {
        class SegmentationYarpPort;
    }
}


class RobotsIO::Utils::SegmentationYarpPort : public RobotsIO::Utils::Segmentation
{
public:

    /**
     * Initialize a Segmentation instance which uses YARP ports to receive segmentation masks and
     * (optionally) provide RGB images to the segmentation module.
     *
     * Required port names are composed as <port_prefix>/segmentation:i and <port_prefix>/rgb:o.
     */
    SegmentationYarpPort(const std::string& port_prefix, const bool& provide_rgb);

    virtual ~SegmentationYarpPort();

    bool reset() override;

    bool is_stepping_required() const override;

    int get_frames_between_iterations() const override;

    std::pair<bool, cv::Mat> segmentation(const bool& blocking) override;

    std::pair<bool, cv::Mat> latest_segmentation() override;

    virtual double get_time_stamp() override;

    virtual void set_rgb_image(const cv::Mat& image, const double& timestamp) override;

private:

    RobotsIO::Utils::YarpBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono>> segmentation_in_;

    RobotsIO::Utils::YarpBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> rgb_out_;

    const bool provide_rgb_;

    cv::Mat cv_rgb_out_;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarp_rgb_out_;

    cv::Mat cv_mask_in_;
    yarp::sig::ImageOf<yarp::sig::PixelMono> yarp_mask_in_;
    double time_stamp_mask_in_;

    const std::string log_name_ = "SegmentationYarpPort";
};

#endif /* ROBOTSIO_SEGMENTATIONYARPPORT_H */

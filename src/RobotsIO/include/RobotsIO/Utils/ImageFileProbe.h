/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_IMAGEFILEPROBE_H
#define ROBOTSIO_IMAGEFILEPROBE_H

#include <RobotsIO/Utils/Data.h>
#include <RobotsIO/Utils/Probe.h>
#include <RobotsIO/Utils/any.h>

#include <opencv2/opencv.hpp>

#include <string>

namespace RobotsIO {
    namespace Utils {
        class ImageFileProbe;
    }
}


class RobotsIO::Utils::ImageFileProbe : public RobotsIO::Utils::Probe
{
public:
    ImageFileProbe(const std::string& output_path, const std::string& prefix, const std::string& output_format);

    virtual ~ImageFileProbe();

protected:
    void on_new_data() override;

private:
    cv::Mat data_cv_;

    std::string output_prefix_;

    const std::string output_format_;

    std::size_t frame_counter_ = 0;

    const std::string log_name_ = "ImageFileProbe";
};

#endif /* ROBOTSIO_IMAGEFILEPROBE_H */

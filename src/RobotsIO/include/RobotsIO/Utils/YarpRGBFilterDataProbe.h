/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_YARPRGBFILTERDATAPROBE_H
#define ROBOTSIO_YARPRGBFILTERDATAPROBE_H

#include <Eigen/Core>

// #include <RobotsIO/Utils/Data.h>
#include <RobotsIO/Utils/Probe.h>
#include <RobotsIO/Utils/RGBFilterData.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>
// #include <RobotsIO/Utils/any.h>

#include <opencv2/opencv.hpp>

#include <string>

namespace RobotsIO {
    namespace Utils {
        class YarpRGBFilterDataProbe;
    }
}


class RobotsIO::Utils::YarpRGBFilterDataProbe : public RobotsIO::Utils::YarpBufferedPort<RobotsIO::Utils::RGBFilterData>,
                                                public RobotsIO::Utils::Probe
{
public:
    YarpRGBFilterDataProbe(const std::string& port_name);

    virtual ~YarpRGBFilterDataProbe();

protected:
    void on_new_data() override;

private:
    cv::Mat image_;

    Eigen::MatrixXd matrix_;
    const std::string log_name_ = "YarpRGBFilterDataProbe";
};

#endif /* ROBOTSIO_YARPRGBFILTERDATAPROBE_H */

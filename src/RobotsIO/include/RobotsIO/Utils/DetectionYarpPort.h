/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DETECTIONYARPPORT_H
#define ROBOTSIO_DETECTIONYARPPORT_H

#include <RobotsIO/Utils/Detection.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>

#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class DetectionYarpPort;
    }
}

class RobotsIO::Utils::DetectionYarpPort : public RobotsIO::Utils::Detection,
                                           public RobotsIO::Utils::YarpBufferedPort<yarp::sig::VectorOf<int>>
{
public:
    DetectionYarpPort(const std::string& port_name);

    virtual ~DetectionYarpPort();

    cv::Rect detection() const override;

    bool freeze(const bool blocking = false) override;

private:
    cv::Rect detection_;
};

#endif /* ROBOTSIO_DETECTIONYARPPORT_H */

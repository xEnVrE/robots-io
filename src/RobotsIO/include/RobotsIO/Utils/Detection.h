/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DETECTION_H
#define ROBOTSIO_DETECTION_H

#include <RobotsIO/Utils/DataStream.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

namespace RobotsIO {
    namespace Utils {
        class Detection;
    }
}


class RobotsIO::Utils::Detection : public RobotsIO::Utils::DataStream
{
public:
    virtual ~Detection();

    virtual cv::Rect detection() const = 0;
};

#endif /* ROBOTSIO_DETECTION_H */

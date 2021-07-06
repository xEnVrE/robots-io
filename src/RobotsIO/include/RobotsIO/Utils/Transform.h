/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_TRANSFORM_H
#define ROBOTSIO_TRANSFORM_H

#include <RobotsIO/Utils/DataStream.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class Transform;
    }
}


class RobotsIO::Utils::Transform : public RobotsIO::Utils::DataStream
{
public:
    virtual ~Transform();

    virtual Eigen::Transform<double, 3, Eigen::Affine> transform() = 0;

    virtual int get_frames_between_iterations() const;

    /**
     * If required, the user might override this method to set the RGB image
     * on which the transform has to be evaluated.
     */
    virtual void set_rgb_image(const cv::Mat& image);
};

#endif /* ROBOTSIO_TRANSFORM_H */

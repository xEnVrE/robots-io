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

    /**
     * N > 1 indicates that the segmentation is available every N frames
     * N = 1 indicates that the segmentation is available at all frames
     * N < 1 indicates that this information is not available
     *
     * By default, this method returns N = 1. User might override this setting by re-implementing this method.
     */
    virtual int get_frames_between_iterations() const;

    /**
     * If required, the user might override this method to set the RGB image
     * on which the transform has to be evaluated.
     */
    virtual void set_rgb_image(const cv::Mat& image);

    /**
     * If required, the user might override this method to set the Depth / Segmentation pair
     * on which the transform has to be evaluated.
     */
    virtual void set_depth_segmentation_image(const Eigen::MatrixXf& depth, const cv::Mat& segmentation);

    /**
     * Indicate whether a new transform has been received or not.
     * Please note that this method might return true even if
     * DataStream::freeze() is false, e.g. if the transform is invalid.
     *
     * User might override this method in order to comunicate
     * when a network-based transform source is ready,
     * independently from the validity of the received transform.
     *
     * The default returned value is True.
     *
     * Warning: this method should be called after DataStream::freeze(),
     * as the reception status might be updated after that.
     */
    virtual bool transform_received();
};

#endif /* ROBOTSIO_TRANSFORM_H */

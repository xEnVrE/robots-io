/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_TRANSFORMYARPPORT_H
#define ROBOTSIO_TRANSFORMYARPPORT_H

#include <RobotsIO/Utils/Transform.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>
#include <RobotsIO/Utils/YarpImageOfMonoFloat.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class TransformYarpPort;
    }
}

class RobotsIO::Utils::TransformYarpPort : public RobotsIO::Utils::Transform,
                                           public RobotsIO::Utils::YarpBufferedPort<yarp::sig::Vector>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Initialize a Transform instance which uses YARP ports to receive transforms and
     * (optionally) provide RGB images, or Depth + Segmentation, to the module which extract transforms.
     *
     * Required port names are composed as <port_prefix>/transform:i, <port_prefix>/rgb:o
     * and <port_prefix>/depth_segmentation:o.
     */
    TransformYarpPort(const std::string& port_prefix, const bool& provide_rgb, const bool& provide_depth_segmentation);

    virtual ~TransformYarpPort();

    Eigen::Transform<double, 3, Eigen::Affine> transform() override;

    Eigen::MatrixXd bounding_box() override;

    bool freeze(const bool blocking = false) override;

    int get_frames_between_iterations() const override;

    void set_rgb_image(const cv::Mat& image) override;

    void set_depth_segmentation_image(const Eigen::MatrixXf& depth, const cv::Mat& segmentation) override;

    bool transform_received() override;

private:
    Eigen::Transform<double, 3, Eigen::Affine> transform_;

    Eigen::MatrixXd bbox_points_;

    RobotsIO::Utils::YarpBufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> rgb_out_;

    RobotsIO::Utils::YarpBufferedPort<RobotsIO::Utils::YarpImageOfMonoFloat> depth_segmentation_out_;

    const bool provide_rgb_;

    const bool provide_depth_segmentation_;

    cv::Mat cv_rgb_out_;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> yarp_rgb_out_;

    cv::Mat cv_depth_out_;
    cv::Mat cv_segmentation_out_;
    RobotsIO::Utils::YarpImageOfMonoFloat yarp_depth_segmentation_out_;

    bool transform_received_ = false;
};

#endif /* ROBOTSIO_TRANSFORMYARPPORT_H */

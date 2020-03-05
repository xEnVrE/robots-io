/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_YARPCAMERA_H
#define ROBOTSIO_YARPCAMERA_H

#include <RobotsIO/Camera/Camera.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <string>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>

namespace RobotsIO {
    namespace Camera {
        class YarpCamera;
    }
}

class RobotsIO::Camera::YarpCamera : public RobotsIO::Camera::Camera
{
public:

    YarpCamera(const std::string& port_prefix, const bool& network_bootstrap = false);

    YarpCamera(const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy, const std::string& port_prefix);

    YarpCamera(const std::string& data_path, const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy);

    ~YarpCamera();

    /**
     * RGB-D and pose.
     */

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> pose(const bool& blocking) override;

    std::pair<bool, cv::Mat> rgb(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> depth(const bool& blocking) override;

    std::pair<bool, double> time_stamp() override;

private:
    yarp::os::Network yarp_;

    /**
     * RGB-D sources.
     */

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_;

    /**
     * Timestamp.
     */

    double time_stamp_;

    bool is_time_stamp_ = false;

    /**
     * Log name to be used in messages printed by the class.
     */

    const std::string log_name_ = "YarpCamera";
};

#endif /* ROBOTSIO_YARPCAMERA_H */

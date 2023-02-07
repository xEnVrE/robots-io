/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
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
#include <yarp/sig/Vector.h>

namespace RobotsIO {
    namespace Camera {
        class YarpCamera;
    }
}

class RobotsIO::Camera::YarpCamera : public RobotsIO::Camera::Camera
{
public:

    YarpCamera(const std::string& port_prefix, const bool& network_bootstrap = false);

    YarpCamera(const std::size_t& width, const std::size_t& height, const double& fx, const double& cx, const double& fy, const double& cy, const std::string& port_prefix, const bool& enable_camera_pose);

    YarpCamera(const std::string& data_path, const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy);

    ~YarpCamera();

    /**
     * RGB-D and pose.
     */

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> pose(const bool& blocking) override;

    std::pair<bool, cv::Mat> rgb(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> depth(const bool& blocking) override;

    std::pair<bool, double> time_stamp_rgb() const override;

    std::pair<bool, double> time_stamp_depth() const override;

private:
    yarp::os::Network yarp_;

    /**
     * RGB-D sources.
     */

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_;

    /**
     * Pose source.
     */

    yarp::os::BufferedPort<yarp::sig::Vector> port_pose_;

    bool enable_camera_pose_ = false;

    yarp::sig::Vector last_camera_pose_;

    /**
     * Timestamp.
     */

    double time_stamp_rgb_;

    double time_stamp_depth_;

    bool is_time_stamp_rgb_ = false;

    bool is_time_stamp_depth_ = false;

    /**
     * Log name to be used in messages printed by the class.
     */

    const std::string log_name_ = "YarpCamera";
};

#endif /* ROBOTSIO_YARPCAMERA_H */

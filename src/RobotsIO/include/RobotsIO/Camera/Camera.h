/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_CAMERA_H
#define ROBOTSIO_CAMERA_H

#include <RobotsIO/Camera/CameraParameters.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <cstdint>
#include <fstream>
#include <string>

namespace RobotsIO {
    namespace Camera {
        class Camera;
    }
}


class RobotsIO::Camera::Camera
{
public:
    Camera();

    virtual ~Camera();

    virtual bool status();

    virtual bool reset();

    /**
     * Camera parameters.
     */

    virtual std::pair<bool, Eigen::MatrixXd> deprojection_matrix() const;

    virtual std::pair<bool, RobotsIO::Camera::CameraParameters> parameters() const;

    /**
     * RGB-D and pose.
     */

    virtual std::pair<bool, Eigen::MatrixXf> depth(const bool& blocking) = 0;

    virtual std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> pose(const bool& blocking) = 0;

    virtual std::pair<bool, cv::Mat> rgb(const bool& blocking) = 0;

    /**
     * Auxiliary data.
     */

    virtual std::pair<bool, Eigen::VectorXd> auxiliary_data(const bool& blocking);

    virtual std::size_t auxiliary_data_size() const;

    /**
     * Offline playback.
     */

    virtual std::int32_t frame_index() const;

    virtual bool is_offline() const;

    virtual bool set_frame_index(const std::int32_t& index);

    virtual bool step_frame();

    /**
     * Logging.
     */

    virtual bool log_frame(const bool& log_depth = false);

    virtual bool start_log(const std::string& path);

    virtual bool stop_log();

protected:
    virtual bool initialize();

    bool status_ = true;

    /**
     * Camera parameters.
     */

    virtual bool evaluate_deprojection_matrix();

    RobotsIO::Camera::CameraParameters parameters_;

    Eigen::MatrixXd deprojection_matrix_;

    bool deprojection_matrix_initialized_ = false;

    /**
     * Constructor for offline playback.
     */
    Camera(const std::string& data_path, const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy);

    /**
     * RGB-D and pose for offline playback.
     */

    virtual std::pair<bool, Eigen::MatrixXf> depth_offline();

    virtual std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> pose_offline();

    virtual std::pair<bool, cv::Mat> rgb_offline();

    /**
     * Auxiliary data for offline playback.
     */

    virtual std::pair<bool, Eigen::VectorXd> auxiliary_data_offline();

    /*
     * Offline playback.
     */

    virtual std::pair<bool, Eigen::MatrixXd> load_data();

    const bool offline_mode_ = false;

    std::string data_path_;

    std::ifstream data_in_;

    Eigen::MatrixXd data_;

    std::int32_t frame_index_ = -1;

    static constexpr std::size_t standard_data_offset_ = 8;

    /*
     * Data logging.
     */

    std::ofstream log_;

    std::string log_path_;

    std::int32_t log_index_ = 0;

    /**
     * Log name to be used in messages printed by the class.
     */

    const std::string log_name_ = "Camera";
};

#endif /* ROBOTSIO_CAMERA_H */

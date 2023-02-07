/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_ICUBCAMERADEPTH_H
#define ROBOTSIO_ICUBCAMERADEPTH_H

#include <RobotsIO/Camera/iCubCameraRelative.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <string>

namespace RobotsIO {
    namespace Camera {
        class iCubCameraDepth;
    }
}


class RobotsIO::Camera::iCubCameraDepth : public RobotsIO::Camera::iCubCameraRelative
{
public:

    iCubCameraDepth(const std::string& robot_name, const std::string& port_prefix, const bool& use_calibration = false, const std::string& calibration_path = "");

    iCubCameraDepth(const std::string& data_path_left, const std::string& data_path_right, const std::size_t& width, const std::size_t& height, const double& fx_l, const double& cx_l, const double& fy_l, const double& cy_l, const double& fx_r, const double& cx_r, const double& fy_r, const double& cy_r, const bool& load_encoders_data, const bool& use_calibration = false, const std::string& calibration_path = "");

    ~iCubCameraDepth();

    /**
     * Camera parameters.
     */

    std::pair<bool, Eigen::MatrixXd> deprojection_matrix() const override;

    /**
     * RGB-D and pose.
     */

    std::pair<bool, Eigen::MatrixXf> depth(const bool& blocking) override;

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> pose(const bool& blocking) override;

    std::pair<bool, cv::Mat> rgb(const bool& blocking) override;

private:
    /**
     * Storage required for stereo matching with OpenCV.
     */

    void configure_sgbm();

    cv::Mat intrinsic_left_;

    cv::Mat distortion_left_;

    cv::Mat intrinsic_right_;

    cv::Mat distortion_right_;

    cv::Ptr<cv::StereoSGBM> sgbm_;

    /**
     * Parameters for OpenCV SGBM.
     * TODO: put them in the constructor somehow
     */
    int uniqueness_ratio_ = 15;

    int speckle_window_size_ = 50;

    int speckle_range_ = 1;

    int number_of_disparities_ = 96;

    int block_size_ = 7;

    int min_disparity_ = 0;

    int pre_filter_cap_ = 63;

    int disp_12_max_diff_ = 0;

    /**
     * Log name to be used in messages printed by the class.
     */

    const std::string log_name_ = "iCubCameraDepth";
};

#endif /* ROBOTSIO_ICUBCAMERADEPTH_H */

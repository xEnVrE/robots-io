/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifdef _OPENMP
#include <omp.h>
#endif

#include <RobotsIO/Camera/iCubCameraDepth.h>

#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

using namespace Eigen;
using namespace RobotsIO::Camera;


iCubCameraDepth::iCubCameraDepth
(
    const std::string& robot_name,
    const std::string& port_prefix,
    const std::string& fallback_context_name,
    const std::string& fallback_configuration_name,
    const bool& use_calibration,
    const std::string& calibration_path
) :
    iCubCameraRelative(robot_name, port_prefix, fallback_context_name, fallback_configuration_name)
{
    configure_sgbm();
}


iCubCameraDepth::iCubCameraDepth
(
    const std::string& data_path_left,
    const std::string& data_path_right,
    const std::size_t& width,
    const std::size_t& height,
    const double& fx_l,
    const double& cx_l,
    const double& fy_l,
    const double& cy_l,
    const double& fx_r,
    const double& cx_r,
    const double& fy_r,
    const double& cy_r,
    const bool& load_encoders_data,
    const bool& use_calibration,
    const std::string& calibration_path
) :
    iCubCameraRelative(data_path_left, data_path_right, width, height, fx_l, cx_l, fy_l, cy_l, fx_r, cx_r, fy_r, cy_r, load_encoders_data, use_calibration, calibration_path)
{
    configure_sgbm();
}


iCubCameraDepth::~iCubCameraDepth()
{}


std::pair<bool, Eigen::MatrixXd> iCubCameraDepth::deprojection_matrix() const
{
    /* Since the depth is aligned with left camera, the left camera parameters are returned here. */
    return get_relative_camera().deprojection_matrix();
}


std::pair<bool, Eigen::MatrixXf> iCubCameraDepth::depth(const bool& blocking)
{
    /* Get the images. */
    bool valid_rgb = false;
    cv::Mat rgb_left;
    cv::Mat rgb_right;
    std::tie(valid_rgb, rgb_left) = get_relative_camera().rgb(blocking);
    if (!valid_rgb)
        return std::make_pair(false, MatrixXf());

    valid_rgb = false;
    std::tie(valid_rgb, rgb_right) = iCubCameraRelative::rgb(blocking);
    if (!valid_rgb)
        return std::make_pair(false, MatrixXf());

    /* Get the extrinsic matrix. */
    bool valid_pose = false;
    Transform<double, 3, Affine> pose;
    std::tie(valid_pose, pose) = iCubCameraRelative::pose(blocking);
    if (!valid_pose)
        return std::make_pair(false, MatrixXf());
    /* As required by SGBM. */
    pose = pose.inverse();

    /* Set the extrinsic matrix in OpenCV format. */
    MatrixXd translation = pose.translation();
    cv::Mat R;
    cv::Mat t;
    cv::eigen2cv(translation, t);
    cv::eigen2cv(pose.rotation(), R);

    /* Perform rectification. */
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;
    cv::stereoRectify(intrinsic_left_, distortion_left_,
                      intrinsic_right_, distortion_right_,
                      rgb_left.size(),
                      R, t,
                      R1, R2, P1, P2, Q, -1);

    cv::Mat mapl0;
    cv::Mat mapl1;
    cv::Mat mapr0;
    cv::Mat mapr1;
    cv::initUndistortRectifyMap(intrinsic_left_, distortion_left_, R1, P1, rgb_left.size(), CV_32FC1, mapl0, mapl1);
    cv::initUndistortRectifyMap(intrinsic_right_, distortion_right_, R2, P2, rgb_left.size(), CV_32FC1, mapr0, mapr1);

    cv::Mat rgb_left_rect;
    cv::Mat rgb_right_rect;
    cv::remap(rgb_left, rgb_left_rect, mapl0, mapl1, cv::INTER_LINEAR);
    cv::remap(rgb_right, rgb_right_rect, mapr0, mapr1, cv::INTER_LINEAR);

    /* Compute disparity. */
    cv::Mat disparity;
    sgbm_->compute(rgb_left_rect, rgb_right_rect, disparity);

    /* Compute mapping from coordinates in the original left image to coordinates in the rectified left image. */
    cv::Mat map(disparity.rows * disparity.cols, 1, CV_32FC2);
    for (int v = 0; v < disparity.rows; v++)
    {
        for (int u = 0; u < disparity.cols; u++)
        {
            map.ptr<float>(v * disparity.cols + u)[0] = float(u);
            map.ptr<float>(v * disparity.cols + u)[1] = float(v);
        }
    }
    cv::undistortPoints(map, map, intrinsic_left_, distortion_left_, R1, P1);

    /* Store some values required for the next computation. */
    float q_00 = float(Q.at<double>(0, 0));
    float q_03 = float(Q.at<double>(0, 3));
    float q_11 = float(Q.at<double>(1, 1));
    float q_13 = float(Q.at<double>(1, 3));
    float q_23 = float(Q.at<double>(2, 3));
    float q_32 = float(Q.at<double>(3, 2));
    float q_33 = float(Q.at<double>(3, 3));
    float r_02 = float(R1.at<double>(0, 2));
    float r_12 = float(R1.at<double>(1, 2));
    float r_22 = float(R1.at<double>(2, 2));

    /* Compute depth. */
    MatrixXf depth(rgb_left.rows, rgb_left.cols);
#pragma omp parallel for collapse(2)
    for (int v = 0; v < rgb_left.rows; v++)
        for (int u = 0; u < rgb_left.cols; u++)
        {
            /* Take coordinates in the rectified image. */
            float u_map = map.ptr<float>(v * disparity.cols +  u)[0];
            float v_map = map.ptr<float>(v * disparity.cols +  u)[1];

            /* Convert to int. */
            int u_r = cvRound(u_map);
            int v_r = cvRound(v_map);

            if ((u_r < 0) || (u_r >= disparity.cols) || (v_r < 0) || ( v_r >= disparity.rows))
            {
                depth(v, u) = std::numeric_limits<double>::infinity();
                continue;
            }

            /* Get disparity. */
            float disparity_value = disparity.at<short>(v_r, u_r) / 16.0;

            /* Evaluate depth. */
            depth(v, u) = (r_02 * (float(u_r) * q_00 + q_03) + r_12 * (float(v_r) * q_11 + q_13) + r_22 * q_23) / (disparity_value * q_32 + q_33);
        }

    return std::make_pair(true, depth);
}


std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> iCubCameraDepth::pose(const bool& blocking)
{
    /* Since the depth is aligned with left camera, the left camera pose is returned here. */
    return get_relative_camera().pose(blocking);
}


std::pair<bool, cv::Mat> iCubCameraDepth::rgb(const bool& blocking)
{
    /* Since the depth is aligned with left camera, the left image is returned here. */
    return get_relative_camera().rgb(blocking);
}


void iCubCameraDepth::configure_sgbm()
{
    /* Get intrinsic parameters of both cameras .*/
    bool valid_parameters = false;
    CameraParameters parameters_left;
    std::tie(valid_parameters, parameters_left) = get_relative_camera().parameters();
    if (!valid_parameters)
    {
        throw(std::runtime_error(log_name_ + "::configure_sgbm. Error: cannot get intrinsic parameters of left camera."));
    }

    valid_parameters = false;
    CameraParameters parameters_right;
    std::tie(valid_parameters, parameters_right) = parameters();
    if (!valid_parameters)
    {
        throw(std::runtime_error(log_name_ + "::configure_sgbm. Error: cannot get intrinsic parameters of right camera."));
    }

    /* Configure intrinsics for OpenCV. */

    intrinsic_left_ = cv::Mat::eye(3,3,CV_64FC1);
    intrinsic_left_.at<double>(0,0) = parameters_left.fx;
    intrinsic_left_.at<double>(0,2) = parameters_left.cx;
    intrinsic_left_.at<double>(1,1) = parameters_left.fy;
    intrinsic_left_.at<double>(1,2) = parameters_left.cy;

    intrinsic_right_ = cv::Mat::eye(3,3,CV_64FC1);
    intrinsic_right_.at<double>(0,0) = parameters_right.fx;
    intrinsic_right_.at<double>(0,2) = parameters_right.cx;
    intrinsic_right_.at<double>(1,1) = parameters_right.fy;
    intrinsic_right_.at<double>(1,2) = parameters_right.cy;

    /* Configure distortion for OpenCV.
     We expect that the images are already undistorted. */
    distortion_left_ = cv::Mat::zeros(1,8,CV_64FC1);
    distortion_left_.at<double>(0,0) = 0.0;
    distortion_left_.at<double>(0,1) = 0.0;
    distortion_left_.at<double>(0,2) = 0.0;
    distortion_left_.at<double>(0,3) = 0.0;

    distortion_right_ = cv::Mat::zeros(1,8,CV_64FC1);
    distortion_right_.at<double>(0,0) = 0.0;
    distortion_right_.at<double>(0,1) = 0.0;
    distortion_right_.at<double>(0,2) = 0.0;
    distortion_right_.at<double>(0,3) = 0.0;

    /* Initialize OpenCV SGBM. */
    sgbm_ = cv::StereoSGBM::create(min_disparity_, number_of_disparities_, block_size_, 8 * 3 * block_size_ * block_size_, 32 * 3 * block_size_ * block_size_, disp_12_max_diff_, pre_filter_cap_, uniqueness_ratio_, speckle_window_size_, speckle_range_, cv::StereoSGBM::MODE_HH);
}

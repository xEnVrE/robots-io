/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/DatasetCamera.h>

using namespace RobotsIO::Camera;


DatasetCamera::DatasetCamera
(
    const std::string& data_path,
    const std::string& data_prefix,
    const std::string& rgb_prefix,
    const std::string& depth_prefix,
    const std::string& data_format,
    const std::string& rgb_format,
    const std::string& depth_format,
    const std::size_t& heading_zeros,
    const std::size_t& index_offset,
    const std::size_t& width,
    const double& height,
    const double& fx,
    const double& cx,
    const double& fy,
    const double& cy
) :
    Camera(data_path, width, height, fx, cx, fy, cy)
{
    /* Set dataset parameters. */
    dataset_parameters_.data_prefix(data_prefix);
    dataset_parameters_.rgb_prefix(rgb_prefix);
    dataset_parameters_.depth_prefix(depth_prefix);
    dataset_parameters_.data_format(data_format);
    dataset_parameters_.rgb_format(rgb_format);
    dataset_parameters_.depth_format(depth_format);
    dataset_parameters_.heading_zeros(heading_zeros);
    dataset_parameters_.index_offset(index_offset);

    Camera::initialize();
}


DatasetCamera::~DatasetCamera()
{ }


std::pair<bool, Eigen::MatrixXf> DatasetCamera::depth(const bool& blocking)
{
    return depth_offline();
}


std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> DatasetCamera::pose(const bool& blocking)
{
    return pose_offline();
}


std::pair<bool, cv::Mat> DatasetCamera::rgb(const bool& blocking)
{
    return rgb_offline();
}


std::pair<bool, double> DatasetCamera::time_stamp_rgb() const
{
    return time_stamp_rgb_offline();
}


std::pair<bool, double> DatasetCamera::time_stamp_depth() const
{
    return time_stamp_depth_offline();
}

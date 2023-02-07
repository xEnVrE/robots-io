/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASETCAMERA_H
#define ROBOTSIO_DATASETCAMERA_H

#include <RobotsIO/Camera/Camera.h>

#include <string>

namespace RobotsIO {
    namespace Camera {
        class DatasetCamera;
    }
}


class RobotsIO::Camera::DatasetCamera : public RobotsIO::Camera::Camera
{
public:
    DatasetCamera(const std::string& data_path, const std::string& data_prefix, const std::string& rgb_prefix, const std::string& depth_prefix, const std::string& data_format, const std::string& rgb_format, const std::string& depth_format, const std::size_t& heading_zeros, const std::size_t& index_offset, const std::size_t& width, const double& height, const double& fx, const double& cx, const double& fy, const double& cy);

    virtual ~DatasetCamera();

    /**
     * RGB-D and pose.
     */

    virtual std::pair<bool, Eigen::MatrixXf> depth(const bool& blocking) override;

    virtual std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> pose(const bool& blocking) override;

    virtual std::pair<bool, cv::Mat> rgb(const bool& blocking) override;

    virtual std::pair<bool, double> time_stamp_rgb() const override;

    virtual std::pair<bool, double> time_stamp_depth() const override;

private:
    const std::string log_name_ = "DatasetCamera";
};

#endif /* ROBOTSIO_DATASETCAMERA_H */

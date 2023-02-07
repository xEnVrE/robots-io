/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_SEGMENTATIONCAMERA_H
#define ROBOTSIO_SEGMENTATIONCAMERA_H

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Camera/CameraParameters.h>
#include <RobotsIO/Utils/Transform.h>

#include <SuperimposeMesh/SICAD.h>

#include <Eigen/Dense>


namespace RobotsIO {
    namespace Camera {
        class SegmentationCamera;
    }
}



class RobotsIO::Camera::SegmentationCamera
{
public:
    SegmentationCamera(const RobotsIO::Camera::CameraParameters& camera_parameters, const std::string& mesh_path, const double& threshold = 0.01);

    ~SegmentationCamera();

    void add_object();

    /**
     * Segmentation mask given a depth image and an object Eigen::Transform<double, 3, Affine>
     */
    std::pair<bool, cv::Mat> mask(const Eigen::MatrixXf& scene_depth, const Eigen::Transform<double, 3, Eigen::Affine> object_transform);

    /**
     * Segmentation mask given a RobotsIO::Camera::Camera and a RobotsIO::Utils::Transform
     */
    std::pair<bool, cv::Mat> mask(std::shared_ptr<RobotsIO::Camera::Camera> camera, std::shared_ptr<RobotsIO::Utils::Transform> object_transform);

private:
    /**
     * Rendering method.
     */
    std::pair<bool, cv::Mat> render_mask(const Eigen::MatrixXf& scene_depth, const Eigen::Transform<double, 3, Eigen::Affine> object_transform);

    /**
     * Object renderer.
     */
    std::unique_ptr<SICAD> renderer_;

    /**
     * Camera parameters.
     */
    RobotsIO::Camera::CameraParameters parameters_;

    /**
     * Threshold for depth/rendered depth comparison.
     */
    const double threshold_;

    /**
     * Log name to be used in messages printed by the class.
     */
    const std::string log_name_ = "SegmentationCamera";
};

#endif /* ROBOTSIO_SEGMENTATIONCAMERA_H */

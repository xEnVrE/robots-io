/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_SEGMENTATIONCAMERA_H
#define ROBOTSIO_SEGMENTATIONCAMERA_H

#include <RobotsIO/Camera/Camera.h>
#include <RobotsIO/Utils/Transform.h>

#include <SuperimposeMesh/SICAD.h>

/* #include <Eigen/Dense> */

/* #include <limits> */
/* #include <opencv2/opencv.hpp> */

/* #include <cstdint> */
/* #include <fstream> */
/* #include <string> */

namespace RobotsIO {
    namespace Camera {
        class SegmentationCamera;
    }
}



class RobotsIO::Camera::SegmentationCamera
{
public:
    SegmentationCamera(std::shared_ptr<Camera> camera, std::shared_ptr<RobotsIO::Utils::Transform> object_transform, const std::string& mesh_path);

    ~SegmentationCamera();

    void add_object();

    /**
     * Segmentation mask.
     */
    std::pair<bool, cv::Mat> mask();

private:
    /**
     * Pointer to the underlying camera.
     */
    std::shared_ptr<RobotsIO::Camera::Camera> camera_;

    /**
     * Object transform.
     */
    std::shared_ptr<RobotsIO::Utils::Transform> transform_;

    /**
     * Object renderer.
     */
    std::unique_ptr<SICAD> renderer_;

    /**
     * Camera parameters.
     */
    std::size_t width_;

    std::size_t height_;

    /**
     * Log name to be used in messages printed by the class.
     */
    const std::string log_name_ = "SegmentationCamera";
};

#endif /* ROBOTSIO_SEGMENTATIONCAMERA_H */

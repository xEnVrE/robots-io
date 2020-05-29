/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/SegmentationCamera.h>
#include <RobotsIO/Camera/CameraParameters.h>

#include <Eigen/Dense>

using namespace RobotsIO::Camera;
using namespace Eigen;


SegmentationCamera::SegmentationCamera(const CameraParameters& camera_parameters, const std::string& object_mesh_path) :
    parameters_(camera_parameters)
{
    /* Configure renderer. */
    SICAD::ModelPathContainer model;
    model["object"] = object_mesh_path;
    renderer_ = std::unique_ptr<SICAD>
    (
        new SICAD(model, parameters_.width(), parameters_.height(), parameters_.fx(), parameters_.fy(), parameters_.cx(), parameters_.cy(), 1)
    );
    renderer_->setOglToCam({1.0, 0.0, 0.0, static_cast<float>(M_PI)});
}


SegmentationCamera::~SegmentationCamera()
{}


std::pair<bool, cv::Mat> SegmentationCamera::mask(const MatrixXf& scene_depth, const Transform<double, 3, Affine> object_transform)
{
    return render_mask(scene_depth, object_transform);
}


std::pair<bool, cv::Mat> SegmentationCamera::mask(std::shared_ptr<Camera> camera, std::shared_ptr<RobotsIO::Utils::Transform> object_transform)
{
    /* Get object pose. */
    if (!object_transform->freeze(true))
    {
        std::cout << log_name_ << "::mask. Error: cannot get object pose." << std::endl;
        return std::make_pair(false, cv::Mat());
    }
    Transform<double, 3, Affine> transform = object_transform->transform();

    /* Get camera depth. */
    bool valid_depth = false;
    Eigen::MatrixXf depth;
    std::tie(valid_depth, depth) = camera->depth(true);
    if (!valid_depth)
    {
        std::cout << log_name_ << "::mask. Error: cannot get depth." << std::endl;
        return std::make_pair(false, cv::Mat());
    }

    return render_mask(depth, transform);
}

std::pair<bool, cv::Mat> SegmentationCamera::render_mask(const Eigen::MatrixXf& scene_depth, const Eigen::Transform<double, 3, Affine> object_transform)
{
    /* Compose pose for renderer. */
    SICAD::ModelPose pose;
    pose.push_back(object_transform.translation()(0));
    pose.push_back(object_transform.translation()(1));
    pose.push_back(object_transform.translation()(2));

    AngleAxisd rotation(object_transform.rotation());
    Vector3d axis = rotation.axis();
    pose.push_back(axis(0));
    pose.push_back(axis(1));
    pose.push_back(axis(2));
    pose.push_back(rotation.angle());

    SICAD::ModelPoseContainer pose_container;
    pose_container.emplace("object", pose);

    /* Placeholders */
    cv::Mat placeholder;
    double cam_x [4] = {0.0, 0.0, 0.0};
    double cam_o [4] = {1.0, 0.0, 0.0, 0.0};

    /* Render depth. */
    cv::Mat rendered_depth;
    if (!(renderer_->superimpose(pose_container, cam_x, cam_o, placeholder, rendered_depth)))
    {
        std::cout << log_name_ << "::render_mask. Error: cannot render depth." << std::endl;
        return std::make_pair(false, cv::Mat());
    }

    /* Use depth to produce a rendered segmentation mask. */
    cv::Mat mask;
#if CV_MAJOR_VERSION >= 4
        cv::threshold(rendered_depth, mask, 0.001, 255, cv::THRESH_BINARY);
#else
        cv::threshold(rendererd_depth, mask, 0.001, 255, CV_THRESH_BINARY);
#endif
    mask.convertTo(mask, CV_8UC1);

    /* Remove pixels that are not coherent with the depth of the entire scene. */
    cv::Mat non_zero_coordinates;
    findNonZero(mask, non_zero_coordinates);
    for (std::size_t i = 0; i < non_zero_coordinates.total(); i++)
    {
        const cv::Point& p = non_zero_coordinates.at<cv::Point>(i);

        if (std::abs(scene_depth(p.y, p.x) - rendered_depth.at<float>(p)) > 0.03)
            mask.at<uchar>(p) = 0.0;
    }

    return std::make_pair(true, mask);
}

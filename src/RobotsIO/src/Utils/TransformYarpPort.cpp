/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/TransformYarpPort.h>

#include <opencv2/core/eigen.hpp>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::cv;
using namespace yarp::eigen;
using namespace yarp::sig;


TransformYarpPort::TransformYarpPort(const std::string& port_prefix, const bool& provide_rgb, const bool& provide_depth_segmentation) :
    YarpBufferedPort<yarp::sig::Vector>(port_prefix + "/transform:i"),
    rgb_out_(port_prefix + "/rgb:o"),
    depth_segmentation_out_(port_prefix + "/depth_segmentation:o"),
    provide_rgb_(provide_rgb),
    provide_depth_segmentation_(provide_depth_segmentation)
{}


TransformYarpPort:: ~TransformYarpPort()
{}


Eigen::Transform<double, 3, Affine> TransformYarpPort::transform()
{
    return transform_;
}


Eigen::MatrixXd TransformYarpPort::bounding_box()
{
    return bbox_points_;
}


bool TransformYarpPort::freeze(const bool blocking)
{
    yarp::sig::Vector* data_yarp = receive_data(blocking);
    transform_received_ = (data_yarp != nullptr);

    if (!transform_received_)
        return false;

    bool invalid_pose = true;
    for (std::size_t i = 0; i < 7; i++)
        invalid_pose &= ((*data_yarp)(i) == 0.0);
    if (invalid_pose)
        return false;

    transform_ = Translation<double, 3>(toEigen(*data_yarp).head<3>());
    AngleAxisd rotation((*data_yarp)(6), toEigen(*data_yarp).segment<3>(3));
    transform_.rotate(rotation);

    // FIXME: this might be moved somewhere else.
    if (data_yarp->size() > 7)
    {
        Eigen::VectorXd bbox_points_data = toEigen(*data_yarp).segment<24>(7);
        bbox_points_.resize(3, 8);
        for (std::size_t i = 0; i < 8; i++)
            bbox_points_.col(i) = bbox_points_data.segment<3>(3 * i);
    }

    return true;
}


int TransformYarpPort::get_frames_between_iterations() const
{
    return -1;
}


void TransformYarpPort::set_rgb_image(const cv::Mat& image)
{
    if (!provide_rgb_)
        return;

    cv_rgb_out_ = image.clone();
    yarp_rgb_out_ = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(cv_rgb_out_);

    rgb_out_.send_data(yarp_rgb_out_);
}


void TransformYarpPort::set_depth_segmentation_image(const Eigen::MatrixXf& depth, const cv::Mat& segmentation)
{
    if (!provide_depth_segmentation_)
        return;

    cv::Mat depth_temp;
    cv::eigen2cv(depth, depth_temp);
    cv_depth_out_ = depth_temp.clone();

    // cv_depth_out_ = cv::Mat(depth.rows(), depth.cols(), CV_32FC1);
    // Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> cv_depth_out_eigen(cv_depth_out_.ptr<float>(), depth.rows(), depth.cols());
    // cv_depth_out_eigen = depth;

    cv_segmentation_out_ = segmentation.clone();

    yarp_depth_segmentation_out_.image_mono = yarp::cv::fromCvMat<PixelMono>(cv_segmentation_out_);
    yarp_depth_segmentation_out_.image_float = yarp::cv::fromCvMat<PixelFloat>(cv_depth_out_);

    depth_segmentation_out_.send_data(yarp_depth_segmentation_out_);
}


bool TransformYarpPort::transform_received()
{
    return transform_received_;
}

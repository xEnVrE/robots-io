/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/YarpCamera.h>
#include <RobotsIO/Utils/ParametersYarpPort.h>

#include <iostream>
#include <thread>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Value.h>

using namespace Eigen;
using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace yarp::cv;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;


YarpCamera::YarpCamera(const std::string& port_prefix, const bool& network_bootstrap)
{
    /* Check YARP network. */
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));
    }

    /* Open rgb input port. */
    if (!(port_rgb_.open("/" + port_prefix + "/rgb:i")))
    {
        std::string err = log_name_ + "::ctor. Error: cannot open rgb input port.";
        throw(std::runtime_error(err));
    }

    /* Open depth input port. */
    if (!(port_depth_.open("/" + port_prefix + "/depth:i")))
    {
        std::string err = log_name_ + "::ctor. Error: cannot open depth input port.";
        throw(std::runtime_error(err));
    }

    if (network_bootstrap)
    {
        /* Read camera parameters from network. */
        ParametersYarpPort network_parameters("/" + port_prefix + "/camera_parameters:i");
        while (!(network_parameters.receive_parameters()))
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            yInfo() << log_name_ + "::ctor. Waiting for camera parameters on port " + "/" + port_prefix + "/dataset/camera_parameters:i";
        }
        parameters_ = CameraParameters(network_parameters);

        Camera::initialize();

        /* Log parameters. */
        std::cout << log_name_ + "::ctor. Camera parameters:" << std::endl;
        std::cout << log_name_ + "    - width: " << parameters_.width() << std::endl;
        std::cout << log_name_ + "    - height: " << parameters_.height() << std::endl;
        std::cout << log_name_ + "    - fx: " << parameters_.fx() << std::endl;
        std::cout << log_name_ + "    - fy: " << parameters_.fy() << std::endl;
        std::cout << log_name_ + "    - cx: " << parameters_.cx() << std::endl;
        std::cout << log_name_ + "    - cy: " << parameters_.cy() << std::endl;
    }
}


YarpCamera::YarpCamera
(
    const std::size_t& width,
    const std::size_t& height,
    const double& fx,
    const double& cx,
    const double& fy,
    const double& cy,
    const std::string& port_prefix,
    const bool& enable_camera_pose
) :
    enable_camera_pose_(enable_camera_pose)
{
    /* Check YARP network. */
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));
    }

    /* Store parameters. */
    parameters_.width(width);
    parameters_.height(height);
    parameters_.fx(fx);
    parameters_.cx(cx);
    parameters_.fy(fy);
    parameters_.cy(cy);
    parameters_.initialized(true);

    /* Open rgb input port. */
    if (!(port_rgb_.open("/" + port_prefix + "/rgb:i")))
    {
        std::string err = log_name_ + "::ctor. Error: cannot open rgb input port.";
        throw(std::runtime_error(err));
    }

    /* Open depth input port. */
    if (!(port_depth_.open("/" + port_prefix + "/depth:i")))
    {
        std::string err = log_name_ + "::ctor. Error: cannot open depth input port.";
        throw(std::runtime_error(err));
    }

    /* OPen camera pose input port. */
    if (enable_camera_pose_)
    {
        if (!(port_pose_.open("/" + port_prefix + "/pose:i")))
        {
            std::string err = log_name_ + "::ctor. Error: cannot open pose input port.";
            throw(std::runtime_error(err));
        }
    }

    Camera::initialize();

    /* Log parameters. */
    std::cout << log_name_ + "::ctor. Camera parameters:" << std::endl;
    std::cout << log_name_ + "    - width: " << parameters_.width() << std::endl;
    std::cout << log_name_ + "    - height: " << parameters_.height() << std::endl;
    std::cout << log_name_ + "    - fx: " << parameters_.fx() << std::endl;
    std::cout << log_name_ + "    - fy: " << parameters_.fy() << std::endl;
    std::cout << log_name_ + "    - cx: " << parameters_.cx() << std::endl;
    std::cout << log_name_ + "    - cy: " << parameters_.cy() << std::endl;
}

YarpCamera::YarpCamera
(
    const std::string& data_path,
    const std::size_t& width,
    const double& height,
    const double& fx,
    const double& cx,
    const double& fy,
    const double& cy
) :
    Camera(data_path, width, height, fx, cx, fy, cy)
{
    Camera::initialize();
}


YarpCamera::~YarpCamera()
{
    /* Close ports. */
    port_rgb_.close();

    port_depth_.close();

    if (enable_camera_pose_)
        port_pose_.close();
}


std::pair<bool, MatrixXf> YarpCamera::depth(const bool& blocking)
{
    ImageOf<PixelFloat>* image_in;
    image_in = port_depth_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, MatrixXf());

    Stamp stamp;
    port_depth_.getEnvelope(stamp);
    time_stamp_depth_ = stamp.getTime();
    is_time_stamp_depth_ = true;

    cv::Mat image = yarp::cv::toCvMat(*image_in);
    Map<Eigen::Matrix<float, Dynamic, Dynamic, Eigen::RowMajor>> float_image(image.ptr<float>(), image.rows, image.cols);

    return std::make_pair(true, float_image);
}


std::pair<bool, Transform<double, 3, Affine>> YarpCamera::pose(const bool& blocking)
{
    if (enable_camera_pose_)
    {
        yarp::sig::Vector* pose_in = port_pose_.read(blocking);

        if (pose_in != nullptr)
        {
            last_camera_pose_ = *pose_in;
        }

        if (last_camera_pose_.size() == 7)
        {
            /* If available, always return the last camera pose. */
            Transform<double, 3, Affine> pose;
            pose = Translation<double, 3>(Vector3d(last_camera_pose_[0], last_camera_pose_[1], last_camera_pose_[2]));
            pose.rotate(AngleAxisd(last_camera_pose_[6], Vector3d(last_camera_pose_[3], last_camera_pose_[4], last_camera_pose_[5])));

            return std::make_pair(true, pose);
        }
        else
        {
            /* Camera pose enabled but not available, return (false, empty). */
            return std::make_pair(false, Transform<double, 3, Affine>());
        }
    }
    else
    {
        /* Camera pose not enabled, always return (true, identity). */
        return std::make_pair(true, Transform<double, 3, Affine>::Identity());
    }
}


std::pair<bool, cv::Mat> YarpCamera::rgb(const bool& blocking)
{
    ImageOf<PixelRgb>* image_in;
    image_in = port_rgb_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, cv::Mat());

    Stamp stamp;
    port_rgb_.getEnvelope(stamp);
    time_stamp_rgb_ = stamp.getTime();
    is_time_stamp_rgb_ = true;

    cv::Mat image = yarp::cv::toCvMat(*image_in);
    cv::resize(image, image, cv::Size(parameters_.width(), parameters_.height()));

    return std::make_pair(true, image);
}


std::pair<bool, double> YarpCamera::time_stamp_rgb() const
{
    return std::make_pair(is_time_stamp_rgb_, time_stamp_rgb_);
}


std::pair<bool, double> YarpCamera::time_stamp_depth() const
{
    return std::make_pair(is_time_stamp_depth_, time_stamp_depth_);
}

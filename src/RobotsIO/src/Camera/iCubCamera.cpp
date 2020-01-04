/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/iCubCamera.h>

#include <iostream>

#include <unsupported/Eigen/MatrixFunctions>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Image.h>

using namespace Eigen;
using namespace RobotsIO::Camera;
using namespace iCub::iKin;
using namespace yarp::cv;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;


iCubCamera::iCubCamera
(
    const std::string& robot_name,
    const std::string& laterality,
    const std::string& port_prefix,
    const std::string& fallback_context_name,
    const std::string& fallback_configuration_name,
    const bool& use_calibration,
    const std::string& calibration_path
) :
    laterality_(laterality),
    use_calibration_(use_calibration)
{
    /* Check YARP network. */
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: YARP network is not available."));
    }

    /* Check laterality. */
    if ((laterality_ != "left") && (laterality_ != "right"))
    {
        std::string err = log_name_ + "::ctor. Please use a valid laterality when constructing the iCubCamera instance.";
        throw(std::runtime_error(err));
    }

    /* Prepare properties for the GazeController. */
    Property properties;
    properties.put("device", "gazecontrollerclient");
    properties.put("remote", "/iKinGazeCtrl");
    properties.put("local", "/" + port_prefix + "/gazecontroller");

    /* Open driver. */
    bool ok = driver_gaze_.open(properties) && driver_gaze_.view(gaze_control_) && (gaze_control_ != nullptr);

    if (ok)
    {
        /* Retrieve camera parameters. */
        Bottle info;
        std::string key;
        gaze_control_->getInfo(info);

        key = "camera_width_" + laterality_;
        if (info.find(key).isNull())
        {
            std::string err = log_name_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera width.";
            throw(std::runtime_error(err));
        }
        parameters_.width(info.find(key).asInt());

        key = "camera_height_" + laterality_;
        if (info.find(key).isNull())
        {
            std::string err = log_name_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera height.";
            throw(std::runtime_error(err));
        }
        parameters_.height(info.find(key).asInt());

        key = "camera_intrinsics_" + laterality_;
        if (info.find(key).isNull())
        {
            std::string err = log_name_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera intrinsic parameters.";
            throw(std::runtime_error(err));
        }
        Bottle *list = info.find(key).asList();
        parameters_.fx(list->get(0).asDouble());
        parameters_.cx(list->get(2).asDouble());
        parameters_.fy(list->get(5).asDouble());
        parameters_.cy(list->get(6).asDouble());

        parameters_.initialized(true);
    }
    else
    {
        /* Stick to encoders .*/
        use_driver_gaze_ = false;

        /* TODO: take parameters from a configuration file. */
        parameters_.width(640);
        parameters_.height(480);
        if (laterality_ == "left")
        {
            parameters_.fx(468.672);
            parameters_.cx(323.045);
            parameters_.fy(467.73);
            parameters_.cy(245.784);
        }
        else
        {
            parameters_.fx(468.488);
            parameters_.cx(301.274);
            parameters_.fy(467.427);
            parameters_.cy(245.503);
        }
        parameters_.initialized(true);

        /* Configure torso. */
        Property properties;
        properties.put("device", "remote_controlboard");
        properties.put("local", "/" + port_prefix + "/torso:i");
        properties.put("remote", "/" + robot_name + "/torso");
        ok = drv_torso_.open(properties) && drv_torso_.view(itorso_) && (itorso_ != nullptr);
        if (!ok)
        {
            std::string err = log_name_ + "::ctor. Error: cannot open remote control board for torso.";
            throw(std::runtime_error(err));
        }

        /* Configure forward kinematics. */
        left_eye_kinematics_ = iCubEye("left_v2");
        right_eye_kinematics_ = iCubEye("right_v2");

        left_eye_kinematics_.setAllConstraints(false);
        right_eye_kinematics_.setAllConstraints(false);

        left_eye_kinematics_.releaseLink(0);
        left_eye_kinematics_.releaseLink(1);
        left_eye_kinematics_.releaseLink(2);
        right_eye_kinematics_.releaseLink(0);
        right_eye_kinematics_.releaseLink(1);
        right_eye_kinematics_.releaseLink(2);
    }

    /* Configure head.
       We require this anyway in order to provide inputs to the calibration model. */
    properties.put("device", "remote_controlboard");
    properties.put("local", "/" + port_prefix + "/head:i");
    properties.put("remote", "/" + robot_name + "/head");
    ok = drv_head_.open(properties) && drv_head_.view(ihead_) && (ihead_ != nullptr);
    if (!ok)
    {
        std::string err = log_name_ + "::ctor. Error: cannot open remote control board for head.";
        throw(std::runtime_error(err));
    }

    /* Load calibration if requested. */
    if (use_calibration_)
        if (!load_calibration_model(calibration_path))
        {
            std::string err = log_name_ + "::ctor. Error: cannot load calibration model.";
            throw(std::runtime_error(err));
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


iCubCamera::iCubCamera
(
    const std::string& data_path,
    const std::string& laterality,
    const std::size_t& width,
    const size_t& height,
    const double& fx,
    const double& cx,
    const double& fy,
    const double& cy,
    const bool& load_encoders_data,
    const bool& use_calibration,
    const std::string& calibration_path
) :
    Camera(data_path, width, height, fx, cx, fy, cy),
    laterality_(laterality),
    load_encoders_data_(load_encoders_data),
    use_calibration_(use_calibration)
{
    /* Load calibration if requested. */
    if (use_calibration_)
        if (!load_calibration_model(calibration_path))
        {
            std::string err = log_name_ + "::ctor. Error: cannot load calibration model.";
            throw(std::runtime_error(err));
        }

    Camera::initialize();
}


iCubCamera::~iCubCamera()
{
    /* Close driver. */
    if (use_driver_gaze_)
      driver_gaze_.close();
    else
    {
        drv_torso_.close();
        drv_head_.close();
    }

    /* Close ports. */
    port_rgb_.close();

    port_depth_.close();
}


bool iCubCamera::look_at(const Eigen::Vector3d& fixation_point)
{
    if (use_driver_gaze_)
    {
        yarp::sig::Vector vector(3);
        toEigen(vector) = fixation_point;

        return gaze_control_->lookAtFixationPointSync(vector);
    }

    return false;
}


bool iCubCamera::stop_motion()
{
    if (use_driver_gaze_)
        return gaze_control_->stopControl();

    return false;
}


std::pair<bool, MatrixXf> iCubCamera::depth(const bool& blocking)
{
    if (is_offline())
        return Camera::depth_offline();

    ImageOf<PixelFloat>* image_in;
    image_in = port_depth_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, MatrixXf());

    cv::Mat image = yarp::cv::toCvMat(*image_in);
    Map<Eigen::Matrix<float, Dynamic, Dynamic, Eigen::RowMajor>> depth(image.ptr<float>(), image.rows, image.cols);

    return std::make_pair(true, depth);
}


std::pair<bool, Transform<double, 3, Affine>> iCubCamera::pose(const bool& blocking)
{
    bool valid_pose = false;
    Transform<double, 3, Affine> pose;

    if (is_offline())
        std::tie(valid_pose, pose) = Camera::pose_offline();
    else
        std::tie(valid_pose, pose) = laterality_pose(laterality_, blocking);

    if (!valid_pose)
        return std::make_pair(false, Transform<double, 3, Affine>());

    /* If calibration was loaded and eye encoders are available, correct pose of right eye. */
    if ((laterality() == "right") && use_calibration_)
    {
        bool valid_encoders_input = false;
        if ((ihead_ != nullptr) || (load_encoders_data_))
        {
            /* Set input. */
            Eigen::VectorXd head_encoders(6);
            std::tie(valid_encoders_input, head_encoders) = auxiliary_data(true);

            /* Corrrect pose of the righe eye. */
            if (valid_encoders_input)
            {
                yarp::sig::Vector input(3);
                toEigen(input) = head_encoders.tail<3>() * M_PI / 180.0;

                /* Get prediction. */
                yarp::sig::Vector prediction = calibration_.predict(input).getPrediction();

                /* Convert to SE3. */
                Eigen::Transform<double, 3, Eigen::Affine> output = exp_map(yarp::eigen::toEigen(prediction));

                pose = pose * output;
            }
        }

        if (!valid_encoders_input)
            std::cout << log_name_ + "::pose. Warning: calibration requested, however eyes encoders cannot be retrieved." << std::endl;
    }

    return std::make_pair(true, pose);
}


std::pair<bool, cv::Mat> iCubCamera::rgb(const bool& blocking)
{
    if (is_offline())
        return Camera::rgb_offline();

    ImageOf<PixelRgb>* image_in;
    image_in = port_rgb_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, cv::Mat());

    cv::Mat image = yarp::cv::toCvMat(*image_in);

    return std::make_pair(true, image);
}


std::pair<bool, Eigen::VectorXd> iCubCamera::auxiliary_data(const bool& blocking)
{
    if (is_offline())
        return Camera::auxiliary_data_offline();

    /* Gaze driver do not provides additional information from encoders. */
    if (use_driver_gaze_)
        return std::make_pair(false, VectorXd());

    yarp::sig::Vector torso_encoders_(3);
    yarp::sig::Vector head_encoders_(6);

    if (!itorso_->getEncoders(torso_encoders_.data()))
        return std::make_pair(false, VectorXd());

    if (!ihead_->getEncoders(head_encoders_.data()))
        return std::make_pair(false, VectorXd());

    VectorXd encoders(9);
    encoders.head<3>() = toEigen(torso_encoders_);
    encoders.tail<6>() = toEigen(head_encoders_);

    return std::make_pair(true, encoders);
}


std::size_t iCubCamera::auxiliary_data_size() const
{
    /* Auxiliary data are torso (3) and head (6) encoders. */
    if (load_encoders_data_)
        return 3 + 6;

    return 0;
}


std::string iCubCamera::laterality()
{
    return laterality_;
}


std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> iCubCamera::laterality_pose(const std::string& laterality, const bool& blocking)
{
    Transform<double, 3, Affine> pose;

    yarp::sig::Vector position_yarp;
    yarp::sig::Vector orientation_yarp;

    bool ok = getLateralityEyePose(laterality, position_yarp, orientation_yarp);

    if (!ok)
        return std::make_pair(false, Transform<double, 3, Affine>());

    pose = Translation<double, 3>(toEigen(position_yarp));
    pose.rotate(AngleAxisd(orientation_yarp(3), toEigen(orientation_yarp).head<3>()));

    return std::make_pair(true, pose);
}


void iCubCamera::set_laterality(const std::string& laterality)
{
    laterality_ = laterality;
}


bool iCubCamera::getLateralityEyePose(const std::string& laterality, yarp::sig::Vector& position, yarp::sig::Vector& orientation)
{
    if ((laterality != "left") && (laterality != "right"))
            return false;

    if (use_driver_gaze_)
    {
        if (laterality == "left")
            return gaze_control_->getLeftEyePose(position, orientation);
        else
            return gaze_control_->getRightEyePose(position, orientation);
    }
    else
    {
        yarp::sig::Vector torso_encoders_(3);
        yarp::sig::Vector head_encoders_(6);

        if (!itorso_->getEncoders(torso_encoders_.data()))
            return false;

        if (!ihead_->getEncoders(head_encoders_.data()))
            return false;

        yarp::sig::Vector chain_joints(8);
        chain_joints(0) = torso_encoders_(2);
        chain_joints(1) = torso_encoders_(1);
        chain_joints(2) = torso_encoders_(0);
        chain_joints(3) = head_encoders_(0);
        chain_joints(4) = head_encoders_(1);
        chain_joints(5) = head_encoders_(2);
        chain_joints(6) = head_encoders_(3);

        double version = head_encoders_(4);
        double vergence = head_encoders_(5);

        if (laterality == "left")
            chain_joints(7) = version + vergence / 2.0;
        else
            chain_joints(7) = version - vergence / 2.0;

        yarp::sig::Vector pose;
        if (laterality == "left")
            pose = left_eye_kinematics_.EndEffPose(chain_joints * M_PI / 180.0);
        else
            pose = right_eye_kinematics_.EndEffPose(chain_joints * M_PI / 180.0);

        position.resize(3);
        orientation.resize(4);
        position = pose.subVector(0, 2);
        orientation = pose.subVector(3, 6);

        return true;
    }
}


Eigen::Transform<double, 3, Eigen::Affine> iCubCamera::exp_map(const Eigen::VectorXd& se3)
{
    Eigen::Transform<double, 3, Eigen::Affine> SE3;

    Eigen::Matrix3d log_R = Eigen::Matrix3d::Zero();
    log_R(0, 1) = -1.0 * se3(5);
    log_R(0, 2) = se3(4);
    log_R(1, 0) = se3(5);
    log_R(1, 2) = -1.0 * se3(3);
    log_R(2, 0) = -1.0 * se3(4);
    log_R(2, 1) = se3(3);

    double theta = se3.tail<3>().norm() + std::numeric_limits<double>::epsilon();
    Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * log_R + (theta - std::sin(theta)) / (std::pow(theta, 3)) * log_R * log_R;

    SE3 = Eigen::Translation<double, 3>(V * se3.head<3>());
    SE3.rotate(log_R.exp());

    return SE3;
}


bool iCubCamera::load_calibration_model(const std::string& model_path)
{
    std::ifstream model_in;
    model_in.open(model_path);
    if (!model_in.is_open())
        return false;

    Bottle model;
    std::stringstream ss;
    ss << model_in.rdbuf();
    model.fromString(ss.str());
    model_in.close();

    calibration_.readBottle(model);

    return true;
}

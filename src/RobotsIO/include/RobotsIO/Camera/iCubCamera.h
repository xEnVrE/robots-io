/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_ICUBCAMERA_H
#define ROBOTSIO_ICUBCAMERA_H

#include <RobotsIO/Camera/Camera.h>

#include <Eigen/Dense>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/learningMachine/LSSVMLearner.h>

#include <opencv2/opencv.hpp>

#include <string>

#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>

namespace RobotsIO {
    namespace Camera {
        class iCubCamera;
    }
}


class RobotsIO::Camera::iCubCamera : public RobotsIO::Camera::Camera
{
public:

    iCubCamera(const std::string& robot_name, const std::string& laterality, const std::string& port_prefix, const bool& use_calibration = false, const std::string& calibration_path = "");

    iCubCamera(const std::string& data_path, const std::string& laterality, const std::size_t& width, const std::size_t& height, const double& fx, const double& cx, const double& fy, const double& cy, const bool& load_encoders_data, const bool& use_calibration = false, const std::string& calibration_path = "");

    ~iCubCamera();

    /**
     * Gaze control interface.
     */

    bool is_controller_available();

    yarp::dev::IGazeControl& controller();

    /**
     * RGB-D and pose.
     */

    std::pair<bool, Eigen::MatrixXf> depth(const bool& blocking) override;

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> pose(const bool& blocking) override;

    std::pair<bool, cv::Mat> rgb(const bool& blocking) override;

    std::pair<bool, double> time_stamp_rgb() const override;

    std::pair<bool, double> time_stamp_depth() const override;

    /**
     * Auxiliary data.
     */

    std::pair<bool, Eigen::VectorXd> auxiliary_data(const bool& blocking) override;

    std::size_t auxiliary_data_size() const override;

protected:
    std::string laterality();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> laterality_pose(const std::string& laterality, const bool& blocking);

    void set_laterality(const std::string& laterality);

private:
    std::string laterality_;

    yarp::os::Network yarp_;

    /**
     * RGB-D sources.
     */

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_;

    /**
     * Drivers.
     */

    /* Gateway to iGazeControl::getLeftEyePose() and iGazeControl::getRightEyePose(). */
    bool getLateralityEyePose(const std::string& laterality, yarp::sig::Vector& position, yarp::sig::Vector& orientation);

    yarp::dev::PolyDriver driver_gaze_;

    yarp::dev::IGazeControl* gaze_control_;

    bool use_driver_gaze_ = true;

    /**
     * Fallback interface with encoders.
     */

    yarp::dev::PolyDriver drv_torso_;

    yarp::dev::IEncoders *itorso_;

    yarp::dev::PolyDriver drv_head_;

    yarp::dev::IEncoders *ihead_;

    iCub::iKin::iCubEye left_eye_kinematics_;

    iCub::iKin::iCubEye right_eye_kinematics_;

    /*
     * Extrinsic calibration.
     */

    Eigen::Transform<double, 3, Eigen::Affine> exp_map(const Eigen::VectorXd& se3);

    bool load_calibration_model(const std::string& model_path);

    bool use_calibration_ = false;

    iCub::learningmachine::LSSVMLearner calibration_;

    /*
     * Offline playback.
     */

    bool load_encoders_data_ = false;

    /**
     * Timestamp.
     */

    double time_stamp_rgb_;

    double time_stamp_depth_;

    bool is_time_stamp_rgb_ = false;

    bool is_time_stamp_depth_ = false;

    /**
     * Log name to be used in messages printed by the class.
     */

    const std::string log_name_ = "iCubCamera";
};

#endif /* ROBOTSIO_ICUBCAMERA_H */

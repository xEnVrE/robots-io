/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_ICUBCAMERARELATIVE_H
#define ROBOTSIO_ICUBCAMERARELATIVE_H

#include <RobotsIO/Camera/iCubCamera.h>

namespace RobotsIO {
    namespace Camera {
        class iCubCameraRelative;
    }
}


class RobotsIO::Camera::iCubCameraRelative : public RobotsIO::Camera::iCubCamera
{
public:

    iCubCameraRelative(const std::string& robot_name, const std::string& port_context, const std::string& fallback_context_name, const std::string& fallback_configuration_name, const bool& use_calibration = false, const std::string& calibration_path = "");

    iCubCameraRelative(const std::string& data_path_left, const std::string& data_path_right, const std::size_t& width, const std::size_t& height, const double& fx_l, const double& cx_l, const double& fy_l, const double& cy_l, const double& fx_r, const double& cx_r, const double& fy_r, const double& cy_r, const bool& load_encoders_data, const bool& use_calibration = false, const std::string& calibration_path = "");

    ~iCubCameraRelative();

    /**
     * RGB-D and pose.
     */

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> pose(const bool& blocking) override;

    /**
     * Offline playback.
     */

    bool step_frame() override;

    bool set_frame_index(const std::int32_t& index) override;

protected:
    RobotsIO::Camera::iCubCamera& get_relative_camera();

    const RobotsIO::Camera::iCubCamera& get_relative_camera() const;

private:
    /**
     * Log name to be used in messages printed by the class.
     */
    const std::string log_name_ = "iCubCameraRelative";

    /**
     * In offline mode, we need an additional instance to read data of both cameras.
     */
    std::unique_ptr<RobotsIO::Camera::iCubCamera> left_camera_;
};

#endif /* ROBOTSIO_ICUBCAMERARELATIVE_H */

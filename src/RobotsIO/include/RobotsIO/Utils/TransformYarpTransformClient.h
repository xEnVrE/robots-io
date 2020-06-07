/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_TRANSFORMYARPTRANSFORMCLIENT_H
#define ROBOTSIO_TRANSFORMYARPTRANSFORMCLIENTR_H

#include <RobotsIO/Utils/Transform.h>

#include <Eigen/Dense>

#include <string>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/Network.h>

namespace RobotsIO {
    namespace Utils {
        class TransformYarpTransformClient;
    }
}


class RobotsIO::Utils::TransformYarpTransformClient : public RobotsIO::Utils::Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TransformYarpTransformClient(const std::string& port_prefix, const std::string& source_name, const std::string& target_name);

    virtual ~TransformYarpTransformClient();

    Eigen::Transform<double, 3, Eigen::Affine> transform() override;

    bool freeze(const bool blocking = false) override;

private:
    yarp::os::Network yarp_;

    yarp::dev::PolyDriver drv_transform_client_;

    yarp::dev::IFrameTransform* transform_client_;

    Eigen::Transform<double, 3, Eigen::Affine> transform_;

    const std::string source_name_;

    const std::string target_name_;

    const std::string log_name_ = "TransformYarpTransformClient";
};

#endif /* ROBOTSIO_TRANSFORMYARPTRANSFORMCLIENT_H */

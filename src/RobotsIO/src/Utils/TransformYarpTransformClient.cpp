/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/TransformYarpTransformClient.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/Property.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::eigen;


TransformYarpTransformClient::TransformYarpTransformClient(const std::string& port_prefix, const std::string& source_name, const std::string& target_name) :
    source_name_(source_name),
    target_name_(target_name)
{
    /* FrameTransformClient initialization. */
    yarp::os::Property tf_properties;
    tf_properties.put("device", "transformClient");
    tf_properties.put("local", "/" + port_prefix + "/transform_client");
    tf_properties.put("remote", "/transformServer");

    bool ok = drv_transform_client_.open(tf_properties);
    ok &= (drv_transform_client_.view(transform_client_) && (transform_client_ != nullptr));
    if (!ok)
    {
        throw(std::runtime_error(log_name_ + "::ctor. Error: cannot initialize the FrameTransformClient."));
    }
}


TransformYarpTransformClient:: ~TransformYarpTransformClient()
{}


Eigen::Transform<double, 3, Affine> TransformYarpTransformClient::transform()
{
    return transform_;
}


bool TransformYarpTransformClient::freeze(const bool blocking)
{
    yarp::sig::Matrix transform(4, 4);
    if (!transform_client_->getTransform(target_name_, source_name_, transform))
        return false;

    transform_.matrix() = toEigen(transform);

    return true;
}

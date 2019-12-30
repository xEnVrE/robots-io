/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_TRANSFORMYARPVECTOR_H
#define ROBOTSIO_TRANSFORMYARPVECTOR_H

#include <RobotsIO/Utils/Transform.h>
#include <RobotsIO/Utils/YarpVectorOfProbe.hpp>

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class TransformYarpPort;
    }
}

class RobotsIO::Utils::TransformYarpPort : public RobotsIO::Utils::Transform,
                                           public RobotsIO::Utils::YarpVectorOfProbe<double>
{
public:
    TransformYarpPort(const std::string& port_name);

    virtual ~TransformYarpPort();

    std::pair<bool ,Eigen::Transform<double, 3, Eigen::Affine>> transform(const bool& blocking = false) override;
};

#endif /* ROBOTSIO_TRANSFORMYARPVECTOR_H */

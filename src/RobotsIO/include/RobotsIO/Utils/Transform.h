/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_TRANSFORM_H
#define ROBOTSIO_TRANSFORM_H

#include <Eigen/Dense>

namespace RobotsIO {
    namespace Utils {
        class Transform;
    }
}

class RobotsIO::Utils::Transform
{
public:
    virtual std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> transform(const bool& blocking = false) = 0;
};

#endif /* ROBOTSIO_TRANSFORM_H */

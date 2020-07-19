/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_FLOATMATRIX_H
#define ROBOTSIO_FLOATMATRIX_H

#include <Eigen/Dense>

#include <RobotsIO/Utils/DataStream.h>

namespace RobotsIO {
    namespace Utils {
        class FloatMatrix;
    }
}


class RobotsIO::Utils::FloatMatrix : public RobotsIO::Utils::DataStream
{
public:
    virtual ~FloatMatrix();

    virtual Eigen::MatrixXf matrix() = 0;

    virtual Eigen::MatrixXd matrix_as_double();
};

#endif /* ROBOTSIO_FLOATMATRIX_H */

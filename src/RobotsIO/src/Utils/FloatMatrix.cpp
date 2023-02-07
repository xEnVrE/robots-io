/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/FloatMatrix.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


FloatMatrix::~FloatMatrix()
{}


MatrixXd FloatMatrix::matrix_as_double()
{
    return matrix().cast<double>();
}

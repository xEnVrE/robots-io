/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Parameters2YarpBottle.h>
#include <RobotsIO/Utils/YarpBottleProbe.hpp>

using namespace RobotsIO::Camera;
using namespace RobotsIO::Utils;
using namespace yarp::os;


template <>
Bottle YarpBottleProbe<CameraParameters>::convert_from(const CameraParameters& data)
{
    Parameters2YarpBottle parameters_2_bottle(data);
    return parameters_2_bottle.extract_to_bottle();
}


template <>
Bottle YarpBottleProbe<DatasetParameters>::convert_from(const DatasetParameters& data)
{
    Parameters2YarpBottle parameters_2_bottle(data);
    return parameters_2_bottle.extract_to_bottle();
}

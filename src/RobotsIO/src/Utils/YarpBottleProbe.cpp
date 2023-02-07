/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Parameters2YarpBottle.h>
#include <RobotsIO/Utils/YarpBottleProbe.hpp>

using namespace RobotsIO::Utils;
using namespace yarp::os;


template <>
void RobotsIO::Utils::YarpBottleProbe<RobotsIO::Utils::Parameters>::on_new_data()
{
    data_ = convert_from(*(RobotsIO::Utils::any_cast<const Parameters*>(get_data())));

    this->send_data(data_);
}


template <>
Bottle YarpBottleProbe<Parameters>::convert_from(const Parameters& data)
{
    Parameters2YarpBottle parameters_2_bottle(data);
    return parameters_2_bottle.extract_to_bottle();
}

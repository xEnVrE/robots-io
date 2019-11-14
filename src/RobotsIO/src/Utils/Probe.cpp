/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Probe.h>

using namespace RobotsIO::Utils;


Probe::~Probe()
{}


void Probe::set_data(const Data& data)
{
    data_ = data;

    /* Signal that new data has been set. */
    on_new_data();
}


Data& Probe::get_data()
{
    return data_;
}

/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Clock.h>

#include <chrono>
#include <thread>

using namespace RobotsIO::Utils;


Clock::~Clock()
{}


double Clock::now() const
{
    auto current_time = std::chrono::steady_clock::now();
    auto since_epoch = std::chrono::duration<double>(current_time.time_since_epoch());
    return since_epoch.count();
}


void Clock::delay(const int& milliseconds) const
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

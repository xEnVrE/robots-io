/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/ClockedComponent.h>

using namespace RobotsIO::Utils;


ClockedComponent::ClockedComponent()
{
    /* Initialize default clock. */
    clock_ = std::make_shared<Clock>();
}


void ClockedComponent::start_count()
{
    current_time_ = clock_->now();
}


double ClockedComponent::stop_count() const
{
    return clock_->now() - current_time_;
}


Clock& ClockedComponent::clock()
{
    return *clock_;
}


void ClockedComponent::replace_clock(std::shared_ptr<Clock> clock)
{
    clock_ = clock;
}

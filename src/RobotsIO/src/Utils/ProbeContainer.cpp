/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/ProbeContainer.h>

using namespace RobotsIO::Utils;


ProbeContainer::~ProbeContainer()
{}


bool ProbeContainer::is_probe(const std::string& name) const
{
    return (probes_.find(name) != probes_.end());
}


Probe& ProbeContainer::get_probe(const std::string& name) const
{
    return *(probes_.at(name));
}

void ProbeContainer::set_probe(const std::string& name, std::unique_ptr<Probe> probe)
{
    probes_[name] = std::move(probe);
}

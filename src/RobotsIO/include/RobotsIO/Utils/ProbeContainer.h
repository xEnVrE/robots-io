/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_PROBECONTAINER_H
#define ROBOTSIO_PROBECONTAINER_H

#include <RobotsIO/Utils/Probe.h>

#include <string>
#include <unordered_map>

namespace RobotsIO {
    namespace Utils {
        class ProbeContainer;
    }
}


class RobotsIO::Utils::ProbeContainer
{
public:
    virtual ~ProbeContainer();

    bool is_probe(const std::string& name) const;

    RobotsIO::Utils::Probe& get_probe(const std::string& name) const;

    void set_probe(const std::string& name, std::unique_ptr<RobotsIO::Utils::Probe> probe);

protected:
    std::unordered_map<std::string, std::unique_ptr<RobotsIO::Utils::Probe>> probes_;

    const std::string log_name_ = "ProbeContainer";
};

#endif /* ROBOTSIO_PROBECONTAINER_H */

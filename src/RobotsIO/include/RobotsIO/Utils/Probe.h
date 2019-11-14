/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_PROBE_H
#define ROBOTSIO_PROBE_H

#include <RobotsIO/Utils/Data.h>

#include <string>

namespace RobotsIO {
    namespace Utils {
        class Probe;
    }
}

class RobotsIO::Utils::Probe
{
public:
    virtual ~Probe();

    void set_data(const RobotsIO::Utils::Data&);

    RobotsIO::Utils::Data& get_data();

protected:
    virtual void on_new_data() = 0;

private:
    RobotsIO::Utils::Data data_;

    const std::string log_name_ = "Probe";
};

#endif /* ROBOTSIO_PROBE_H */

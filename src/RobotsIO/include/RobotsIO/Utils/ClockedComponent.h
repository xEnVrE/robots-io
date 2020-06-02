/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_CLOCKEDCOMPONENT_H
#define ROBOTSIO_CLOCKEDCOMPONENT_H

#include <RobotsIO/Utils/Clock.h>

#include <memory>

namespace RobotsIO {
    namespace Utils {
        class ClockedComponent;
    }
}

class RobotsIO::Utils::ClockedComponent
{
public:
    ClockedComponent();

    void start_count();

    double stop_count() const;

    Clock& clock();

    void replace_clock(std::shared_ptr<Clock> clock);

private:
    std::shared_ptr<RobotsIO::Utils::Clock> clock_;

    double current_time_;
};

#endif /* ROBOTSIO_CLOCKEDCOMPONENT_H */

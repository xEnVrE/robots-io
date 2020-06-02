/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_CLOCK_H
#define ROBOTSIO_CLOCK_H

namespace RobotsIO {
    namespace Utils {
        class Clock;
    }
}

class RobotsIO::Utils::Clock
{
public:
    virtual ~Clock();

    virtual double now() const;

    virtual void delay(const int& milliseconds) const;
};

#endif /* ROBOTSIO_CLOCK_H */

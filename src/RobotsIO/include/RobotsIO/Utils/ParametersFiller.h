/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_PARAMETERS_FILLER_H
#define ROBOTSIO_PARAMETERS_FILLER_H

#include <string>

namespace RobotsIO {
    namespace Utils {
        class ParametersFiller;
    }
}


class RobotsIO::Utils::ParametersFiller
{
public:
    virtual const std::pair<bool, std::string> fill_string(const std::string& key) const = 0;

    virtual const std::pair<bool, std::size_t> fill_size_t(const std::string& key) const = 0;

    virtual const std::pair<bool, int> fill_int(const std::string& key) const = 0;

    virtual const std::pair<bool, double> fill_double(const std::string& key) const = 0;

    virtual const std::pair<bool, bool> fill_bool(const std::string& key) const = 0;
};

#endif /* ROBOTSIO_PARAMETERS_FILLER_H */

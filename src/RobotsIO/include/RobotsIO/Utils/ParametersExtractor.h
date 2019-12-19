/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_PARAMETERS_EXTRACTOR_H
#define ROBOTSIO_PARAMETERS_EXTRACTOR_H

#include <string>

namespace RobotsIO {
    namespace Utils {
        class ParametersExtractor;
        class Parameters;
    }
}


class RobotsIO::Utils::ParametersExtractor
{
public:
    ParametersExtractor(const RobotsIO::Utils::Parameters& parameters);

    void extract_fields();

    void extract_field(const std::string& key);

    virtual void extract_field(const std::string& key, const std::string& value) = 0;

    virtual void extract_field(const std::string& key, const int& value) = 0;

    virtual void extract_field(const std::string& key, const double& value) = 0;

    virtual void extract_field(const std::string& key, const bool& value) = 0;

protected:
    const RobotsIO::Utils::Parameters& parameters_;
};

#endif /* ROBOTSIO_PARAMETERS_EXTRACTOR_H */

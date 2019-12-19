/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/ParametersExtractor.h>
#include <RobotsIO/Utils/Parameters.h>

using namespace RobotsIO::Utils;


ParametersExtractor::ParametersExtractor(const RobotsIO::Utils::Parameters& parameters) :
    parameters_(parameters)
{ }


void ParametersExtractor::extract_fields()
{
    for (const auto& key : parameters_.keys())
    {
        parameters_.extract_field(key, *this);
    }
}


void ParametersExtractor::extract_field(const std::string& key)
{
    parameters_.extract_field(key, *this);
}

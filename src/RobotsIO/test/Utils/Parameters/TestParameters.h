/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/Parameters.h>
#include <RobotsIO/Utils/ParametersExtractor.h>

class TestParameters : public RobotsIO::Utils::Parameters
{
public:
    TestParameters();

    virtual ~TestParameters();

    robots_io_accessor(TestParameters);

    robots_io_declare_field(TestParameters, int, field0);

    robots_io_declare_field(TestParameters, double, field1);

    robots_io_declare_field(TestParameters, bool, field2);

    robots_io_declare_std_field(TestParameters, string, field3);

    robots_io_declare_std_field(TestParameters, size_t, field4);
};

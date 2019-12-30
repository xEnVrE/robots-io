/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <TestParameters.h>

robots_io_accessor_impl(TestParameters);

robots_io_declare_field_impl(TestParameters, int, field0);

robots_io_declare_field_impl(TestParameters, double, field1);

robots_io_declare_field_impl(TestParameters, bool, field2);

robots_io_declare_std_field_impl(TestParameters, string, field3);

robots_io_declare_std_field_impl(TestParameters, size_t, field4);

TestParameters::TestParameters()
{
    field0(0);

    field1(0.0);

    field2(true);

    field3("");

    field4(0);
}

TestParameters::~TestParameters()
{}

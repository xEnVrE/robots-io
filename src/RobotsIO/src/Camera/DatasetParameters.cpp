/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Camera/DatasetParameters.h>

using namespace RobotsIO::Camera;

robots_io_accessor_impl(DatasetParameters);

robots_io_declare_std_field_impl(DatasetParameters, string, path);

robots_io_declare_std_field_impl(DatasetParameters, string, data_prefix);

robots_io_declare_std_field_impl(DatasetParameters, string, rgb_prefix);

robots_io_declare_std_field_impl(DatasetParameters, string, depth_prefix);

robots_io_declare_std_field_impl(DatasetParameters, string, data_format);

robots_io_declare_std_field_impl(DatasetParameters, string, rgb_format);

robots_io_declare_std_field_impl(DatasetParameters, string, depth_format);

robots_io_declare_std_field_impl(DatasetParameters, size_t, heading_zeros);

robots_io_declare_std_field_impl(DatasetParameters, size_t, index_offset);

robots_io_declare_std_field_impl(DatasetParameters, size_t, standard_data_offset);

robots_io_declare_field_impl(DatasetParameters, bool, pose_available);


DatasetParameters::DatasetParameters()
{
    /* Set default values. */
    data_prefix("");

    rgb_prefix("");

    depth_prefix("");

    data_format("txt");

    rgb_format("png");

    depth_format("float");

    heading_zeros(0);

    index_offset(0);

    standard_data_offset(0);

    pose_available(false);
}


DatasetParameters::~DatasetParameters()
{}

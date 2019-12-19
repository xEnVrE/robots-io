/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASETPARAMETERS_H
#define ROBOTSIO_DATASETPARAMETERS_H

#include <RobotsIO/Utils/Parameters.h>

#include <string>

namespace RobotsIO {
    namespace Camera {
        struct DatasetParameters;
    }
}


class RobotsIO::Camera::DatasetParameters : public RobotsIO::Utils::Parameters
{
public:
    DatasetParameters();

    virtual ~DatasetParameters();

    robots_io_accessor(DatasetParameters);

    robots_io_declare_std_field(DatasetParameters, string, path);

    robots_io_declare_std_field(DatasetParameters, string, data_prefix);

    robots_io_declare_std_field(DatasetParameters, string, rgb_prefix);

    robots_io_declare_std_field(DatasetParameters, string, depth_prefix);

    robots_io_declare_std_field(DatasetParameters, string, data_format);

    robots_io_declare_std_field(DatasetParameters, string, rgb_format);

    robots_io_declare_std_field(DatasetParameters, string, depth_format);

    robots_io_declare_std_field(DatasetParameters, size_t, heading_zeros);

    robots_io_declare_std_field(DatasetParameters, size_t, index_offset);

    robots_io_declare_std_field(DatasetParameters, size_t, standard_data_offset);

    bool pose_available = false;
};

#endif /* ROBOTSIO_DATASETPARAMETERS_H */

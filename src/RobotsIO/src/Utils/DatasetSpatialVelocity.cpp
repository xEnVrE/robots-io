/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/DatasetSpatialVelocity.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


DatasetSpatialVelocity::DatasetSpatialVelocity(const std::string& file_path, const std::size_t& skip_rows, const std::size_t& skip_cols, const std::size_t& expected_cols, const int rx_time_index, const int tx_time_index) :
    DatasetDataStream(file_path, skip_rows, skip_cols, expected_cols, rx_time_index, tx_time_index)
{}


DatasetSpatialVelocity::~DatasetSpatialVelocity()
{}


double DatasetSpatialVelocity::elapsed_time()
{
    return elapsed_time_;
}


VectorXd DatasetSpatialVelocity::twist()
{
    return twist_;
}


bool DatasetSpatialVelocity::freeze(const bool blocking)
{
    if (!DatasetDataStream::freeze())
        return false;

    /* Twist data. */
    twist_ = data();

    /* Elapsed time. */
    double rx_time = DatasetDataStream::rx_time();
    elapsed_time_ = 0.0;
    if (last_time_initialized_)
        elapsed_time_ = rx_time - last_time_;
    last_time_ = rx_time;
    last_time_initialized_ = true;

    return true;
}

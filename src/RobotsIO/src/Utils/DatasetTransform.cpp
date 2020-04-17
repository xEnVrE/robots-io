/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/DatasetTransform.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


DatasetTransform::DatasetTransform
(
    const std::string& file_path,
    const std::size_t& skip_rows,
    const std::size_t& skip_cols,
    const std::size_t& expected_cols,
    const int rx_time_index,
    const int tx_time_index
) :
    DatasetDataStream(file_path, skip_rows, skip_cols, expected_cols, rx_time_index, tx_time_index)
{}


DatasetTransform::~DatasetTransform()
{}


Eigen::Transform<double, 3, Eigen::Affine> DatasetTransform::transform()
{
    return transform_;
}


bool DatasetTransform::freeze(const bool blocking)
{
    if (!DatasetDataStream::freeze())
        return false;

    VectorXd transform_data = data();

    bool invalid_pose = true;
    for (std::size_t i = 0; i < transform_data.size(); i++)
        invalid_pose &= (transform_data(i) == 0.0);
    if (invalid_pose)
        return false;

    transform_ = Translation<double, 3>(transform_data.head<3>());
    AngleAxisd rotation(transform_data(6), transform_data.segment<3>(3));
    transform_.rotate(rotation);

    return true;
}

/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/DatasetTransformDelayed.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


DatasetTransformDelayed::DatasetTransformDelayed
(
    const double& fps,
    const double& simulated_fps,
    const bool simulate_inference_time,
    const std::string& file_path,
    const std::size_t& skip_rows,
    const std::size_t& skip_cols,
    const std::size_t& expected_cols,
    const int rx_time_index,
    const int tx_time_index
) :
    DatasetDataStreamDelayed(fps, simulated_fps, simulate_inference_time, file_path, skip_rows, skip_cols, expected_cols, rx_time_index, tx_time_index),
    fps_(fps),
    simulated_fps_(simulated_fps)
{}


DatasetTransformDelayed::~DatasetTransformDelayed()
{}


Eigen::Transform<double, 3, Eigen::Affine> DatasetTransformDelayed::transform()
{
    return transform_;
}


bool DatasetTransformDelayed::freeze(const bool blocking)
{
    if (!DatasetDataStreamDelayed::freeze())
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


int DatasetTransformDelayed::get_frames_between_iterations() const
{
    return int(fps_ / simulated_fps_);
}

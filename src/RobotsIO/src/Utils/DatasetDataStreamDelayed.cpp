/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/DatasetDataStreamDelayed.h>

using namespace Eigen;
using namespace RobotsIO::Utils;


DatasetDataStreamDelayed::DatasetDataStreamDelayed
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
    DatasetDataStream(file_path, skip_rows, skip_cols, expected_cols, rx_time_index, tx_time_index),
    delay_(static_cast<int>(fps / simulated_fps)),
    simulate_inference_time_(simulate_inference_time)
{
    if (simulate_inference_time_)
        set_head(get_head() - delay_ + skip_rows);
}


DatasetDataStreamDelayed::~DatasetDataStreamDelayed()
{}


VectorXd DatasetDataStreamDelayed::data()
{
    int head = get_head();

    if (simulate_inference_time_)
    {
        if (head < 0)
            head = 0;
    }

    return DatasetDataStream::data(head);
}


bool DatasetDataStreamDelayed::freeze()
{
    DatasetDataStream::freeze();

    if ((get_head() % delay_) != 0)
        return false;

    return true;
}

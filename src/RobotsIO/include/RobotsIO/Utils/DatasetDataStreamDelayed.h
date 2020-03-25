/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_DATASETDATASTREAMDELAYED_H
#define ROBOTSIO_DATASETDATASTREAMDELAYED_H

#include <RobotsIO/Utils/DatasetDataStream.h>

namespace RobotsIO {
    namespace Utils {
        class DatasetDataStreamDelayed;
    }
}


class RobotsIO::Utils::DatasetDataStreamDelayed : public RobotsIO::Utils::DatasetDataStream
{
public:
    DatasetDataStreamDelayed(const double& fps, const double& simulated_fps, const bool simulate_inference_time, const std::string& file_path, const std::size_t& skip_rows, const std::size_t& skip_cols, const std::size_t& expected_cols, const int rx_time_index = NoTimeIndex, const int tx_time_index = NoTimeIndex);

    virtual ~DatasetDataStreamDelayed();

    bool freeze() override;

private:
    const int delay_;

    const bool simulate_inference_time_;

    const std::string log_name_ = "DatasetDataStreamDelayed";
};

#endif /* ROBOTSIO_DATASETDATASTREAMDELAYED_H */

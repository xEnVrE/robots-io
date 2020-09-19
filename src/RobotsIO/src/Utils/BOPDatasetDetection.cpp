/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/BOPDatasetDetection.h>

#include <fstream>

#include <json/json.h>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace Json;


BOPDatasetDetection::BOPDatasetDetection(const std::string& sequence_root_path, const std::size_t& object_id) :
    object_id_(object_id)
{
    std::ifstream scene_gt_stream(sequence_root_path + "/scene_gt.json");
    if (!scene_gt_stream.is_open())
        throw(std::runtime_error(log_name_ + "::ctor. Error: failed to open " + sequence_root_path + "/scene_gt.json"));

    std::ifstream scene_gt_info_stream(sequence_root_path + "/scene_gt_info.json");
    if (!scene_gt_info_stream.is_open())
        throw(std::runtime_error(log_name_ + "::ctor. Error: failed to open " + sequence_root_path + "/scene_gt_info.json"));

    CharReaderBuilder builder;
    JSONCPP_STRING errors;

    Value scene_gt_root;
    if (!parseFromStream(builder, scene_gt_stream, &scene_gt_root, &errors))
        throw(std::runtime_error(log_name_ + "::ctor. Error: failed to parse " + sequence_root_path + "/scene_gt.json"));

    Value scene_gt_info_root;
    if (!parseFromStream(builder, scene_gt_info_stream, &scene_gt_info_root, &errors))
        throw(std::runtime_error(log_name_ + "::ctor. Error: failed to parse " + sequence_root_path + "/scene_gt_info.json"));

    number_frames_ = scene_gt_root.size();

    for (std::size_t i = 0; i < number_frames_; i++)
    {
        const auto& frame = scene_gt_root[std::to_string(i + 1)];

        int object_index = 0;
        for (; object_index < frame.size(); object_index++)
        {
            if (frame[object_index]["obj_id"] == int(object_id))
                break;
        }

        if (object_index == frame.size())
            throw(std::runtime_error(log_name_ + "::ctor. Error: found frame with no instance of object "  + std::to_string(object_id)));

        const auto& frame_info = scene_gt_info_root[std::to_string(i + 1)];
        const auto& bounding_box = frame_info[object_index]["bbox_visib"];
        detection_.push_back(cv::Rect(bounding_box[0].asUInt(), bounding_box[1].asUInt(), bounding_box[2].asUInt(), bounding_box[3].asUInt()));
    }

    scene_gt_stream.close();
    scene_gt_info_stream.close();
}


BOPDatasetDetection::~BOPDatasetDetection()
{}


bool BOPDatasetDetection::freeze(const bool blocking)
{
    head_++;

    if (head_ >= number_frames_)
        return false;

    return true;
}


cv::Rect BOPDatasetDetection::detection() const
{
    return detection_.at(head_);
}

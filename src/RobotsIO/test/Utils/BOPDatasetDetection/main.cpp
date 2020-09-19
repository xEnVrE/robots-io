/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RobotsIO/Utils/BOPDatasetDetection.h>

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <string>

using namespace RobotsIO::Utils;


class ParsingTest
{
public:
    ParsingTest(const std::string& sequence_root_path, const std::size_t& object_id) :
        detection_(sequence_root_path, object_id)
    {}

    bool test()
    {
        /* Extract detections. */
        std::vector<cv::Rect> detections;

        while (detection_.freeze())
        {
            detections.push_back(detection_.detection());
        }

        bool ok = true;

        /* Check number of elements. */
        if (detections.size() != number_frames_)
        {
            std::cerr << "Expected " << number_frames_ << " detections. Found " << detections.size() << "." << std::endl;

            ok = false;
        }
        std::cout << detections.size() << std::endl;

        /* Check some elements. */
        auto check_detection = [](const cv::Rect& detection, const int& x, const int& y, const int& width, const int& height)
        {
            bool ok = true;

            if (detection.x != x)
            {
                std::cerr << "Expected x coordinate of detection = " << x << ". Found " << detection.x << "." << std::endl;

                ok = false;
            }

            if (detection.y != y)
            {
                std::cerr << "Expected y coordinate of detection = " << y << ". Found " << detection.y << "." << std::endl;

                ok = false;
            }

            if (detection.width != width)
            {
                std::cerr << "Expected width of detection = " << width << ". Found " << detection.width << "." << std::endl;

                ok = false;
            }

            if (detection.height != height)
            {
                std::cerr << "Expected height of detection = " << height << ". Found " << detection.height << "." << std::endl;

                ok = false;
            }

            return ok;
        };

        const cv::Rect detection_0 = detections.at(0);
        const cv::Rect detection_final = detections.back();
        ok &= check_detection(detection_0, d_0_x, d_0_y, d_0_w, d_0_h);
        ok &= check_detection(detection_final, d_final_x, d_final_y, d_final_w, d_final_h);

        return ok;
    }

private:
    BOPDatasetDetection detection_;

    const std::size_t number_frames_ = 2243;

    const int d_0_x = 282;
    const int d_0_y = 280;
    const int d_0_w = 112;
    const int d_0_h = 73;

    const int d_final_x = 301;
    const int d_final_y = 297;
    const int d_final_w = 126;
    const int d_final_h = 80;
};


int main(int argc, char** argv)
{
    std::string sequence_root_path{"."};
    std::size_t object_id{6};

    ParsingTest test(sequence_root_path, object_id);

    if (!(test.test()))
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}

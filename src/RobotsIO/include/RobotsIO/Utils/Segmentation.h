/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSIO_SEGMENTATION_H
#define ROBOTSIO_SEGMENTATION_H

#include <opencv2/opencv.hpp>

#include <string>

namespace RobotsIO {
    namespace Utils {
        class Segmentation;
    }
}


class RobotsIO::Utils::Segmentation
{
public:

    virtual ~Segmentation();

    virtual bool reset();

    virtual bool step_frame();

    virtual bool is_stepping_required() const = 0;

    virtual void reset_data_loading_time();

    virtual double get_data_loading_time() const;

    /**
     * N > 1 indicates that the segmentation is available every N frames
     * N = 1 indicates that the segmentation is available at all frames
     * N < 1 indicates that this information is not available
     *
     * By default, this method returns N = 1. User might override this setting by re-implementing this method.
     */
    virtual int get_frames_between_iterations() const;

    /**
     * Provide a new segmentation mask.
     */
    virtual std::pair<bool, cv::Mat> segmentation(const bool& blocking) = 0;

    /**
     * Provide the latest valid segmentation mask that has been received.
     *
     * By default, this return (false, cv::Mat()). User might override this method if required.
     */
    virtual std::pair<bool, cv::Mat> latest_segmentation();

    /**
     * If required, the user might override this method to set the RGB image
     * on which the segmentation has to be evaluated.
     */
    virtual void set_rgb_image(const cv::Mat& image);

private:

    const std::string log_name_ = "Segmentation";
};

#endif /* ROBOTSIO_SEGMENTATION_H */

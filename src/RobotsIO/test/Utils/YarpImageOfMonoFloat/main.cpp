#include <RobotsIO/Utils/FileToDepth.h>
#include <RobotsIO/Utils/YarpImageOfMonoFloat.h>
#include <RobotsIO/Utils/YarpBufferedPort.hpp>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <yarp/cv/Cv.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>

#include <chrono>
#include <thread>
#include <iostream>

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std::chrono_literals;


int main(int argc, char** argv)
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        std::cout << "::main. Unable to find YARP." << std::endl;

        return EXIT_FAILURE;
    }
    YarpBufferedPort<YarpImageOfMonoFloat> port_out("/test/image:o");

    bool valid_depth;
    Eigen::MatrixXf depth_eigen;
    std::tie(valid_depth, depth_eigen) = file_to_depth("./depth.float");
    if (valid_depth)
        std::cout << "depth image in" << std::endl;

    cv::Mat depth_image;
    cv::eigen2cv(depth_eigen, depth_image);

    cv::Mat mask_image = cv::imread("./mask.png", cv::IMREAD_COLOR);
    if (!mask_image.empty())
        std::cout << "mask image in " << std::endl;
    cv::cvtColor(mask_image, mask_image, cv::COLOR_BGR2GRAY);

    while (true)
    {
        YarpImageOfMonoFloat images;
        images.image_mono = yarp::cv::fromCvMat<PixelMono>(mask_image);
        images.image_float = yarp::cv::fromCvMat<PixelFloat>(depth_image);

        port_out.send_data(images);

        std::this_thread::sleep_for(1s);
    }
}

namespace yarp RobotsIO.Utils


struct YarpImageOfMono
{
}
(
    yarp.name = "yarp::sig::ImageOf<yarp::sig::PixelMono>"
    yarp.includefile="yarp/sig/Image.h"
)


struct YarpImageOfFloat
{
}
(
    yarp.name = "yarp::sig::ImageOf<yarp::sig::PixelFloat>"
    yarp.includefile="yarp/sig/Image.h"
)


struct YarpImageOfMonoFloat
{
    1: YarpImageOfMono  image_mono;
    2: YarpImageOfFloat image_float;
}
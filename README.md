# RobotsIO

Small C++ library to ease access to some devices of a robot using standard
formats from `Eigen` and `OpenCV`. **Please note that this is a WIP.**

## Dependencies

- [`Eigen 3`](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [`OpenCV`](https://opencv.org/)

### Optional
If the following are enabled, some functionalities related to the iCub robot or to YARP are also available.

- [`ICUB`](https://github.com/robotology/icub-main)
- [`YARP`](https://github.com/robotology/yarp)

## Camera
In namespace `RobotsIO::Camera` classes related to cameras are available. Using these cameras, it is possible to read depth as `Eigen::MatrixXf`, rgb as a `cv::Mat` and the camera pose as a `Eigen::Transform<double, 3, Eigen::Affine>`.

Implemented classes are:
- `Camera`, base class ready to be used if loading data from a disk. The same class provides logging facilities to inheriting classes;
- `CameraParameters`, hosting mostly `width`, `height` and intrinsic parameters of the camera;
- `iCubCamera`, class for the iCub robot inheriting from `Camera` and supporting
  depth and rgb from YARP ports and the camera pose from `IGazeControl` or `IEncoders` or raw YARP ports. It also loads the camera parameters from the `IGazeControl` interface, if available;
- `iCubCameraRelative`, similar to `iCubCamera` but representing the right
  camera with pose expressed relative to the left camera. Useful for experiments dealing with the stereo setup of the robot only;
- `YarpCamera`, class inheriting from `Camera` and supporting depth and rgb from raw YARP ports and parameters from the class constructor.

To be done:
- `RealSense` (using YARP);
- `RealSense` (using ROS);
- `R1Camera` inheriting from `RealSense` (YARP version).

`YARP` is required to build `YarpCamera` and `YARP + ICUB` are required to build `iCubCamera` and `iCubCameraRelative`.

## Hand

The namespace `RobotsIO::Hand` only contains the `iCubHand` class that
provides a measure of the encoders of all the joints of the iCub hand (i.e. of all the fingers).

This class supports reading from `IEncoders` or raw YARP ports and combines all
inputs available (i.e. readings from the hand motors + readings from the analog
Hall sensors mounted on the fingers' joints or only hand motors) to provide a
complete description in the form of `std::unordered_map<std::string,
Eigen::VectorXd>`.  The key is the finger name while the vector contains the encoders values.

The class take advantage of some methods available in `iCub::iKin::iCubFinger` in order to
- combines measurements from motors encoders and analog sensors;
- rescale signals from analog sensors using user provided analog
bounds.

The library provides a template `icub_hand_configuration.ini.template` that can
be filled with the bounds and is installed in a dedicated context within the `iCUBContrib` ecosystem.

`YARP + ICUB` are required to build this class.

To be done:
- an abstract class `Hand`.

## Probes

In namespace `RobotsIO::Utils`, the main classes are `ProbeContainer`
and `Probe`. The idea behind a `ProbeContainer` is to provide any class 
with the ability to store some *probes* in it and retrieve them given
their name. A `Probe` is something that can be used to *send* data somewhere.

```
std::unique_ptr<Probe> probe = (..)
your_class.add_probe("probe_name", std::move(probe));
```

```
(...)

/**
 * Suppose `data` contains some kind of data you need to send to a probe.
 */
if (is_probe("probe_name"))
    get_probe("probe_name")->set_data(data);

(...)
```

The method `Probe::set_data()` accepts data in the form of
`RobotsUtils::Utils::any` that is an implementation of `std::any` working also
with `C++ 11` enabled compilers taken from
[here](https://github.com/robotology/bayes-filters-lib/blob/master/src/BayesFilters/include/BayesFilters/any.h). It
means that it can accept any kind of data, provided data the specific probe you are using knows how to process it.

Available classes are
- `YarpVectorOfProbe<T, class U = yarp::sig::VectorOf<T>>` for
sending data over a `yarp::os::BufferedPort<yarp::sig::VectorOf<T>>` given a
`yarp::sig::VectorOf<T>`. Specializations are also available in order to send
data over the same kind of port starting from `U = Eigen::VectorXd` or `U =
Eigen::Transform<double, 3, Eigen::Affine>` (in the latter case the data sent contains 3D
Cartesian coordintes and an axis-angle representation)
- `YarpImageOfProbe<T>` for sending images over a
`yarp::os::BufferedPort<yarp::sig::ImageOf<T>` starting from
`cv::Mat`. Conversion from OpenCv to YARP images is automatically handled
internally using `yarp::cv::fromCvMat<T>`. `T` can be any kind of
`yarp::sig::Pixel*` given that it is supported by `yarp::cv::fromCvMat<T>`.

Using this kind of probes, it is possible to, e.g., write code in a generic way and,
only if it is required to use YARP ports on a specific system, install `RobotsIO`
with `YARP` support.

To be done:
- Probes for saving images and data on disk
- Probes for ROS

You need `YARP` to build YARP probes.

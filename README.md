# RobotsIO

Small C++ library to ease access to some devices of a robot using standard
formats from `Eigen` and `OpenCV`.
### Dependencies

- [`Eigen 3`](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [`OpenCV`](https://opencv.org/)

#### Optional
If the following are enabled, some functionalities related to the iCub robot and/or YARP are also available.

- [`ICUB`](https://github.com/robotology/icub-main)
- [`YARP`](https://github.com/robotology/yarp)

### Installation

```
git clone https://github.com/xenvre/robots-io.git
cd robots-io
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=<installation_path> [-DUSE_ICUB=ON] [-DUSE_YARP=ON] ../
make install
```

In order to use the library within a `CMake` project
```
find_package(RobotsIO REQUIRED)
(...)
target_link_libraries(... RobotsIO::RobotsIO ...)
```

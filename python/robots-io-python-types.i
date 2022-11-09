%module(directors="1", package="robotsio") robotsio

%include "yarp_import.i"

%include <RobotsIO/Utils/YarpImageOfMonoFloat.h>

%{
#include <RobotsIO/Utils/YarpImageOfMonoFloat.h>
%}

%{
typedef RobotsIO::Utils::YarpImageOfMonoFloat YarpImageOfMonoFloat;
typedef yarp::os::TypedReader<YarpImageOfMonoFloat> TypedReaderYarpImageOfMonoFloat;
typedef yarp::os::TypedReaderCallback<YarpImageOfMonoFloat> TypedReaderCallbackYarpImageOfMonoFloat;
typedef yarp::os::BufferedPort<YarpImageOfMonoFloat> BufferedPortYarpImageOfMonoFloat;
%}

%feature("notabstract") YarpImageOfMonoFloat;
%feature("notabstract") yarp::os::BufferedPort<YarpImageOfMonoFloat>;
%feature("notabstract") BufferedPortYarpImageOfMonoFloat;

%template(TypedReaderYarpImageOfMonoFloat) yarp::os::TypedReader<RobotsIO::Utils::YarpImageOfMonoFloat >;
%template(TypedReaderCallbackYarpImageOfMonoFloat) yarp::os::TypedReaderCallback<RobotsIO::Utils::YarpImageOfMonoFloat >;
%template(BufferedPortYarpImageOfMonoFloat) yarp::os::BufferedPort<RobotsIO::Utils::YarpImageOfMonoFloat >;

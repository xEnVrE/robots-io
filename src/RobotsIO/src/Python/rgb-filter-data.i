%module(package="RobotsIOPythonTypes") RobotsIOPythonTypes

%include "yarp_import.i"

%{
#include <RobotsIO/Utils/RGBFilterData.h>
%}

MAKE_COMMS(RobotsIOUtilsRGBFilterData)

%include <RobotsIO/Utils/RGBFilterData.h>

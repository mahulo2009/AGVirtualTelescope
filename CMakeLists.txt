cmake_minimum_required(VERSION 3.20)
project(AGVirtualTelescope)

set(CMAKE_CXX_STANDARD 14)

include_directories(/opt/gcs/ext/gsl_1.16/include)
link_directories(/opt/gcs/ext/gsl_1.16/lib/linux)

add_executable(AGVirtualTelescope main.cpp AGVirtualTelescope.cpp AGVirtualTelescope.h)

target_link_libraries(AGVirtualTelescope gsl gslcblas)
add_library(AGVirtualTelescope SHARED AGKinematic.cpp AGKinematic.h)

target_include_directories(AGVirtualTelescope PUBLIC /opt/gcs/ext/gsl_1.16/include)
target_link_directories(AGVirtualTelescope PUBLIC /opt/gcs/ext/gsl_1.16/lib/linux)

add_executable(main main.cpp)
target_link_libraries(main AGVirtualTelescope gsl gslcblas)



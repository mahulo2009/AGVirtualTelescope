add_library(AGKinematic SHARED AGKinematic.cpp AGKinematic.h)

target_include_directories(AGKinematic PUBLIC /opt/gcs/ext/gsl_1.16/include)
target_link_directories(AGKinematic PUBLIC /opt/gcs/ext/gsl_1.16/lib/linux)

add_executable(main main.cpp)
target_link_libraries(main AGKinematic gsl gslcblas)



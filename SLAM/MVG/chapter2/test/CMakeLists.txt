cmake_minimum_required(VERSION 2.8.11)
project(measure_construct)

set(CMAKE_CXX_STANDARD 11)

include_directories(/usr/include/eigen3)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

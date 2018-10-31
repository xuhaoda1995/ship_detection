cmake_minimum_required(VERSION 3.5)
project(ShipDetection)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3 REQUIRED)

add_executable(ShipDetection
        main.cpp
        MorphoFilter.cpp
        MorphoFilter.h
        utils.h
        utils.cpp)

target_link_libraries(ShipDetection ${OpenCV_LIBS})
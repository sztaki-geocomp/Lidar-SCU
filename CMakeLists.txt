cmake_minimum_required(VERSION 3.10)

project(Lidar-SCU)

find_package (PCL 1.8 REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)

include_directories(/usr/include/eigen3 /usr/include/pcl-1.8 /usr/include/opencv ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/inc ${CMAKE_CURRENT_SOURCE_DIR}/inc/MRF ${CMAKE_CURRENT_SOURCE_DIR}/src)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
set  (project_SOURCES src/main.cpp src/processMLS.cpp src/sceneInterpreter.cpp src/DataModel.cpp src/Registration.cpp src/rayTracer.cpp src/MRFWrapper.cpp)

set  (project_HEADERS inc/sceneInterpreter.h inc/DataModel.h inc/Registration.h inc/rayTracer.hpp inc/MRFWrapper.h)

set  (OPENCV_LIBRARIES opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_features2d opencv_calib3d opencv_video)

ADD_EXECUTABLE (Lidar-SCU ${project_SOURCES})

TARGET_LINK_LIBRARIES (Lidar-SCU ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OPENCV_LIBRARIES} libMRF.a)

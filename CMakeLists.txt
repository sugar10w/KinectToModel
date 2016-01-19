cmake_minimum_required(VERSION 2.8)
project(KinectToModel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  svm
)
LINK_DIRECTORIES(
    ${PCL_LIBRARY_DIRS}
)
ADD_DEFINITIONS(
    ${PCL_DEFINITIONS}
)

set(LIBRARIES
    ${OPENCV_LIBS}
    ${OPENCV_LIBRARIES}
)
set(SOURCE_FILES
    main.cpp
    PointCloudBuilder.cpp
    KinectParameters.cpp
    LoadRGBD.cpp
    PointCloudMinus.cpp
    LocateCenter.cpp
    Rotate.cpp
    )
add_executable(KinectToModel ${SOURCE_FILES})

target_link_libraries(KinectToModel
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

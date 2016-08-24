cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2)

include(../cmake_config/RoboAutoGlobal.txt)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        rospy
        sensor_msgs
        cmake_modules
        std_msgs
        cv_bridge
        tf
        tf2
        tf2_ros
)


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES orb_slam2
        CATKIN_DEPENDS roscpp message_runtime
        DEPENDS system_lib eigen
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
)

link_directories(
        ${roscpp_LIBRARY_DIRS}
        ${roslib_LIBRARY_DIRS}
        ${catkin_LIBRARY_DIRS}
        ${local_LIBRARY_DIRS}
)

add_library(orb_slam2
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
)

target_link_libraries(orb_slam2
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${roslib_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        DBoW2
        g2o
)

# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum orb_slam2
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${roslib_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti orb_slam2
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${roslib_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum  orb_slam2
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${roslib_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti
        orb_slam2
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${roslib_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS})


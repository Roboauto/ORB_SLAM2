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
        robo_utils
)


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost 1.55 REQUIRED system serialization) # system serialization

catkin_package(
        INCLUDE_DIRS include ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2 ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
        LIBRARIES orb_slam2
        CATKIN_DEPENDS roscpp message_runtime
        DEPENDS system_lib eigen DBoW2 g2o
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
)

link_directories(
        ${roscpp_LIBRARY_DIRS}
        ${roslib_LIBRARY_DIRS}
        ${catkin_LIBRARY_DIRS}
        ${local_LIBRARY_DIRS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib
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
        include/CVSerializationHelper.h src/SerializationTester.cpp include/SerializationTester.h include/BowSerializationHelper.h)

target_link_libraries(orb_slam2
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${roslib_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Boost_LIBRARIES}
        DBoW2
        g2o
)



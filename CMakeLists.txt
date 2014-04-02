cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_BUILD_TYPE Release)

rosbuild_init()

rosbuild_add_boost_directories()

#FIND_PACKAGE( OpenMP REQUIRED)
#if(OPENMP_FOUND)
#message("OPENMP FOUND")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

#SET (CMAKE_C_COMPILER	"/usr/bin/clang")
#SET (CMAKE_CXX_COMPILER "/usr/bin/clang++")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

find_package(ZMQ REQUIRED)
include_directories(${ZMQ_INCLUDE_DIR})

find_package(CZMQ REQUIRED)
include_directories(${CZMQ_INCLUDE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/src/gmapping/)

ADD_DEFINITIONS(-DLINUX -ffast-math)


set (gmapping_src src/gmapping/configfile/configfile.cpp src/gmapping/gridfastslam/gridslamprocessor_tree.cpp src/gmapping/gridfastslam/motionmodel.cpp src/gmapping/gridfastslam/gfsreader.cpp src/gmapping/gridfastslam/gridslamprocessor.cpp src/gmapping/scanmatcher/scanmatcher.cpp src/gmapping/scanmatcher/eig3.cpp src/gmapping/scanmatcher/scanmatcherprocessor.cpp src/gmapping/scanmatcher/smmap.cpp src/gmapping/utils/movement.cpp src/gmapping/utils/stat.cpp src/gmapping/utils/printmemusage.cpp src/gmapping/log/configuration.cpp src/gmapping/log/carmenconfiguration.cpp src/gmapping/log/sensorstream.cpp src/gmapping/log/sensorlog.cpp src/gmapping/sensor/sensor_range/rangereading.cpp src/gmapping/sensor/sensor_range/rangesensor.cpp src/gmapping/sensor/sensor_base/sensor.cpp src/gmapping/sensor/sensor_base/sensorreading.cpp src/gmapping/sensor/sensor_odometry/odometryreading.cpp src/gmapping/sensor/sensor_odometry/odometrysensor.cpp src/protobuf/particle_reading.pb.cc src/protobuf/protobufhelper.cpp)

rosbuild_add_executable(bin/slam_gmapping src/slam_gmapping.cpp src/main.cpp ${gmapping_src})
target_link_libraries(bin/slam_gmapping ${PROTOBUF_LIBRARY} ${ZMQ_LIBRARY} ${CZMQ_LIBRARY})
rosbuild_link_boost(bin/slam_gmapping signals)
#rosbuild_add_executable(tftest src/tftest.cpp)

rosbuild_add_executable(bin/zmq_reqrep_worker src/zmq_reqrep_worker.cpp src/gmapping/scanmatcher/scanmatcher.cpp src/protobuf/particle_reading.pb.cc src/protobuf/protobufhelper.cpp src/gmapping/scanmatcher/smmap.cpp src/gmapping/utils/movement.cpp src/gmapping/utils/stat.cpp)
target_link_libraries(bin/zmq_reqrep_worker ${PROTOBUF_LIBRARY} ${ZMQ_LIBRARY} ${CZMQ_LIBRARY} ${TCMALLOC_LIBRARY})
rosbuild_link_boost(bin/zmq_reqrep_worker  program_options)

#rosbuild_add_executable(test/rtest EXCLUDE_FROM_ALL test/rtest.cpp)
#rosbuild_add_gtest_build_flags(test/rtest)
#rosbuild_declare_test(test/rtest)

# Need to make the tests more robust; currently the output map can differ
# substantially between runs.
#rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/basic_localization_stage_indexed.bag #test/basic_localization_stage_indexed.bag 322a0014f47bcfbb0ad16a317738b0dc)
#rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/hallway_slow_2011-03-04-21-41-33.bag #test/hallway_slow_2011-03-04-21-41-33.bag e772b89713693adc610f4c5b96f5dc03)
#rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/basic_localization_stage_groundtruth.pgm #test/basic_localization_stage_groundtruth.pgm abf208f721053915145215b18c98f9b3)
#rosbuild_add_rostest(test/basic_localization_stage.launch)
#rosbuild_add_rostest(test/basic_localization_laser_different_beamcount.test)

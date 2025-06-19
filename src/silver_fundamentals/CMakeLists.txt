# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(silver_fundamentals)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs create_fundamentals std_msgs
        message_generation geometry_msgs)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "-O2 -march=native -fopenmp")


option(ENABLE_ASAN False)

if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address,undefined)
    add_link_options(-fsanitize=address,undefined)
endif ()


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Message definitions
add_message_files(
  FILES
  Pose.msg
)
#uncomment if you have defined services
add_service_files(
        FILES
        Laser.srv
        LaserCartesian.srv
        DriveData.srv
        ResetEncoders.srv
        ExecutePlan.srv
        Com.srv
        MoveToPosition.srv
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} example/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES std_msgs create_fundamentals geometry_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
        DEPENDS python-serial
        CATKIN_DEPENDS roscpp rospy sensor_msgs create_fundamentals message_runtime geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(example_laser example/example_laser.cpp)
# target_link_libraries(example_laser ${catkin_LIBRARIES})
# add_dependencies(example_laser create_fundamentals_generate_messages_cpp)

# add_executable(example_moving example/example_moving.cpp)
# target_link_libraries(example_moving ${catkin_LIBRARIES})
# add_dependencies(example_moving create_fundamentals_generate_messages_cpp)

# add_executable(wanderer src/wanderer.cpp lib/laser_distance_map.cpp)
# target_link_libraries(wanderer ${catkin_LIBRARIES})
# add_dependencies(wanderer create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

# add_executable(square_with_encoders lib/feedback_drive.cpp lib/coordinate_conversion.cpp src/square_with_encoders.cpp lib/ransac.cpp)
# target_link_libraries(square_with_encoders ${catkin_LIBRARIES})
# add_dependencies(square_with_encoders create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

add_executable(laser_server lib/laser_server.cpp)
target_link_libraries(laser_server ${catkin_LIBRARIES})
add_dependencies(laser_server create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

add_executable(drive_server lib/drive_server.cpp)
target_link_libraries(drive_server ${catkin_LIBRARIES})
add_dependencies(drive_server create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

add_executable(comm_node src/com_node.cpp)
target_link_libraries(comm_node ${catkin_LIBRARIES})
add_dependencies(comm_node create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

# add_executable(square_no_sensors src/square_no_sensors.cpp)
# target_link_libraries(square_no_sensors ${catkin_LIBRARIES})
# add_dependencies(square_no_sensors create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

add_executable(align src/align.cpp lib/feedback_drive.cpp lib/coordinate_conversion.cpp lib/ransac.cpp)
target_link_libraries(align ${catkin_LIBRARIES})
add_dependencies(align create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

add_executable(test lib/feedback_drive.cpp lib/coordinate_conversion.cpp src/test.cpp lib/ransac.cpp lib/parse_map_file.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
add_dependencies(test create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

# add_executable(execute_plan_server src/execute_plan_server.cpp lib/feedback_drive.cpp lib/coordinate_conversion.cpp lib/ransac.cpp)
# target_link_libraries(execute_plan_server ${catkin_LIBRARIES})
# add_dependencies(execute_plan_server create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

add_executable(localize src/localize.cpp lib/parse_map_file.cpp lib/feedback_drive.cpp lib/coordinate_conversion.cpp lib/ransac.cpp lib/play_song.cpp lib/signal_handler.cpp)
target_link_libraries(localize ${catkin_LIBRARIES})
add_dependencies(localize create_fundamentals_generate_messages_cpp silver_fundamentals_generate_messages_cpp)

## Pose server node
add_executable(pose_server src/pose_server.cpp)
target_link_libraries(pose_server ${catkin_LIBRARIES})
add_dependencies(pose_server silver_fundamentals_generate_messages_cpp)

## move to point node
add_executable(move_to_point lib/move_to_point_server.cpp)
target_link_libraries(move_to_point ${catkin_LIBRARIES})
add_dependencies(move_to_point silver_fundamentals_generate_messages_cpp)

# add_executable(play_song lib/play_song.cpp)
# target_link_libraries(play_song ${catkin_LIBRARIES})
# add_dependencies(play_song ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} create_fundamentals_generate_messages_cpp)

cmake_minimum_required (VERSION 3.28.0)

project(carla-ros2-native)

set (LIBCARLA_SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../LibCarla/source)

file (
    GLOB
    LIBCARLA_ROS2_SOURCES
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/listeners/*.cpp
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/types/*.cpp
)

file (
    GLOB
    LIBCARLA_ROS2_HEADERS
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/publishers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/subscribers/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/listeners/*.h
    ${LIBCARLA_SOURCE_PATH}/carla/ros2/types/*.h
)

add_library (carla-ros2-native SHARED
    ${LIBCARLA_ROS2_HEADERS}
    ${LIBCARLA_ROS2_SOURCES}
)

target_include_directories (carla-ros2-native SYSTEM PRIVATE
    ${LIBCARLA_SOURCE_PATH}
    ${CMAKE_INSTALL_PREFIX}/include
    ${BOOST_INCLUDE_PATH}/variant2/include
    ${BOOST_INCLUDE_PATH}/mp11/include
    ${BOOST_INCLUDE_PATH}/assert/include
    ${BOOST_INCLUDE_PATH}/config/include
)

target_compile_definitions (carla-ros2-native PUBLIC
    BOOST_ASIO_ENABLE_BUFFER_DEBUGGING
)

target_link_libraries(carla-ros2-native
  ${CMAKE_INSTALL_PREFIX}/lib/libfastrtps.so)

install (TARGETS carla-ros2-native DESTINATION lib)
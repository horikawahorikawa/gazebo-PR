cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED thread system filesystem regex)
find_package(gazebo REQUIRED)

find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(cloner cloner.cc)
target_link_libraries(cloner ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

set(APPS_DIR ${CMAKE_SOURCE_DIR} CACHE INTERNAL "" FORCE)

add_subdirectory(${APPS_DIR}/quiz/cluster ${APPS_DIR}/quiz/cluster/build)
add_subdirectory(${APPS_DIR}/quiz/ransac ${APPS_DIR}/quiz/ransac/build)

add_executable (environment environment.cpp render/render.cpp processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})


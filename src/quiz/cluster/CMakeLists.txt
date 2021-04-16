cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(quizCluster)
file(GLOB src *.cpp *.h)

#find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


#add_executable (quizCluster cluster.cpp ../../render/render.cpp)
add_executable(${PROJECT_NAME} ${src} ../../render/render.cpp)
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
add_custom_command(TARGET quizCluster POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/libs"
        $<TARGET_FILE_DIR:quizCluster>)




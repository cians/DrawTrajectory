cmake_minimum_required(VERSION 2.8)
project(Viewer)

#add C++11 support
SET(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

#add Pangolin support for OpenGL visualization
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/Viewer)
#add project directory viewer for Qt Creator


add_executable(Viewer Viewer/Main.cpp)
target_link_libraries(Viewer
  ${Pangolin_LIBRARIES}
  )

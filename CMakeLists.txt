# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project
PROJECT(pathplanning)
MESSAGE(${PROJECT_NAME} ":")

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Ofast -g -Wall -fpermissive -Wno-ignored-attributes -Wno-misleading-indentation -Wno-deprecated-declarations -Wno-unused-local-typedefs")


# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
#SET(RobWork_DIR ${RW_ROOT}/cmake)
SET(RobWork_DIR ~/RobWork/RobWork/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# Add the source files to build
SET(SrcFiles src/main.cpp src/LuaGenerator.cpp src/FindPath.hpp)


# The shared library to build:
ADD_EXECUTABLE(pathplanning ${SrcFiles})
TARGET_LINK_LIBRARIES(pathplanning ${ROBWORK_LIBRARIES})

MESSAGE(${PROJECT_NAME} " done!")

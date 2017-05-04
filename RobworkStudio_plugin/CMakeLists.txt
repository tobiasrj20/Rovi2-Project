#####################################################
# Template for building RobWork dependent projects
# - the src should contain code for putting into libraries
# - the plugins contain code for multiple RobWorkStudio plugins
# - the test for testing
# - the example dir for illustrativ examples of your project
#
# Use config.cmake to change default settings and edit this
# file for adding additional dependencies to external projects
#####################################################

#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(PluginUIApp)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_VERBOSE_MAKEFILE True)

# Now set the RW/RWS root (edit this if necessary)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Set the RobWorkStudio root (edit in .bashrc if necessary)
SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})

# If not the -DCMAKE_BUILD_TYPE option is given when running CMake, use Release as default.
IF( NOT DEFINED CMAKE_BUILD_TYPE )
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

#Include default settings for constructing a robwork dependent project
#SET(RobWork_DIR ${RW_ROOT}/cmake)
SET(RobWork_DIR ~/RobWork/RobWork/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
MESSAGE("-- RobWork found: " ${RW_ROOT})

#Include default settings for constructing a robworkstudio dependent project
#SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
SET(RobWorkStudio_DIR ~/RobWork/RobWorkStudio/cmake)
FIND_PACKAGE(RobWorkStudio REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )
MESSAGE("-- RobWorkStudio found: " ${RWSTUDIO_ROOT})

# Set the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}build/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/build/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/build/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)


# if we want to use ui files add them here
SET(UIS_FILES src/SamplePlugin.ui )

IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ELSE()
	QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ENDIF()

#Need to add the current binary dir to the include directory because the generated source files are placed here
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# MOC
IF(RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles src/SamplePlugin.hpp)
ELSE()
	QT4_WRAP_CPP(MocSrcFiles src/SamplePlugin.hpp)
ENDIF()

# Rcc the files:
IF(RWS_USE_QT5)
	QT5_ADD_RESOURCES(RccSrcFiles src/resources.qrc)
ELSE()
	QT4_ADD_RESOURCES(RccSrcFiles src/resources.qrc)
ENDIF()

# Add the source files to build
SET(SrcFiles src/SamplePlugin.cpp ${UIS_OUT_H})

# The shared library to build:
ADD_LIBRARY(SamplePlugin MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(SamplePlugin ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})

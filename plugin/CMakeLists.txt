#\-------------------------------------- . -----------------------------------/#
# Filename : CMakeList.txt               | Smode Laser Plugin                  #
# Author   : Joshua Batty                |                                     #
# Started  : 05/04/2020 18:40            |                                     #
#/-------------------------------------- . -----------------------------------\#

OPTION(SMODE_OPTION_PLUGING_LASER_WITH_SDK "build plugin laser with SDK" ON)

IF (SMODE_OPTION_PLUGING_LASER_WITH_SDK)
  cmake_minimum_required(VERSION 3.0)
  project(Laser CXX)
  set(SMODE_SDK_PATH "C:/Program Files (x86)/SmodeTech/Smode SDK")
  set(CMAKE_MODULE_PATH "${SMODE_SDK_PATH}/CMake")
  find_package(SmodeSDK REQUIRED)
ENDIF ()

SET (LASER_PLUGIN_SOURCES
  LaserPoint.sglsl
  LaserGeometryShader.sglsl
  PointsLaserGeometryShader.sglsl
  LinesLaserGeometryShader.sglsl
  LaserDeviceIdentifier.h
  NannouLaserDeviceIdentifier.h
  NannouLaserDevice.h
  NannouLaserDeviceFactory.h
  LaserDevice.h
  LaserDeviceSelector.h
  LaserInputGeometry.h
  LaserGeometryRenderer.h
)

IF (SMODE_OPTION_PLUGING_LASER_WITH_SDK)
  smode_add_plugin(Laser EDITOR "Laser example plugin" SOURCES ${LASER_PLUGIN_SOURCES})
ELSE ()
  smode_add_default_plugin(Laser SOURCES ${LASER_PLUGIN_SOURCES})
ENDIF ()

# The following 4 system static libs are required by the `smode_laser.lib`.
# These can be discovered by building the rust project with:
# `cargo rustc -- --print=native-static-libs`
target_link_libraries(Laser "advapi32.lib")
target_link_libraries(Laser "ws2_32.lib")
target_link_libraries(Laser "userenv.lib")

# Link to the static library built from our rust project.
set(PLUGIN_DEBUG_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../smode_laser/target/debug")
set(PLUGIN_RELEASE_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../smode_laser/target/release")

target_link_libraries(Laser debug "${PLUGIN_DEBUG_LIB_PATH}/smode_laser.lib" optimized "${PLUGIN_RELEASE_LIB_PATH}/smode_laser.lib")

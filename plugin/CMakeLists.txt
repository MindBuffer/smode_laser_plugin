cmake_minimum_required(VERSION 3.0)
project(Laser CXX)

set(SMODE_SDK_PATH "C:/Program Files (x86)/SmodeTech/Smode SDK")
set(CMAKE_MODULE_PATH "${SMODE_SDK_PATH}/CMake")
find_package(SmodeSDK REQUIRED)

SET (LASER_PLUGIN_SOURCES
  LaserDeviceIdentifier.h
  LaserDevice.h
  LaserDeviceFactory.h
  Laser.h
)

smode_add_plugin(Laser EDITOR "Laser example plugin" SOURCES ${LASER_PLUGIN_SOURCES})

# The following 4 system static libs are required by the `smode_laser.lib`.
# These can be discovered by building the rust project with:
# `cargo rustc -- --print=native-static-libs`
target_link_libraries(Laser "advapi32.lib")
target_link_libraries(Laser "ws2_32.lib")
target_link_libraries(Laser "userenv.lib")
# We don't need to link this one as it's linked by default.
# target_link_libraries(Laser "msvcrt.lib")

# Link to the static library built from our rust project.
set(PLUGIN_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../smode_laser/target/debug")
target_link_libraries(Laser "${PLUGIN_LIB_PATH}/smode_laser.lib")


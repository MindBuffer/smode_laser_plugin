cmake_minimum_required(VERSION 3.0)
project(Laser CXX)

set(SMODE_SDK_PATH "C:/Program Files (x86)/SmodeTech/Smode SDK")
set(CMAKE_MODULE_PATH "${SMODE_SDK_PATH}/CMake")
find_package(SmodeSDK REQUIRED)

smode_add_plugin(Laser
  EDITOR "Laser example plugin"
  SOURCES
    Laser.h)

set(PLUGIN_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../smode_laser/target/release")
target_link_libraries(Laser "${PLUGIN_LIB_PATH}/smode_laser.lib")
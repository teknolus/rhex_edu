cmake_minimum_required(VERSION 3.18.2)
project(pyrhexapi)

find_package(pybind11 CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED
  gstreamer-1.0
  gstreamer-rtp-1.0
  gstreamer-plugins-base-1.0
  gstreamer-video-1.0)
pkg_check_modules(GLIB REQUIRED glib-2.0)

# Ensure RHEX_API_DIR is set
if(NOT DEFINED ENV{RHEX_API_DIR})
  message(FATAL_ERROR "Please set the RHEX_API_DIR environment variable")
endif()

# Define the Pybind11 module
pybind11_add_module(pyrhexapi src/pyrhexapi.cpp)

# Set library output directory
set_target_properties(pyrhexapi PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/src)

# Include directories
target_include_directories(pyrhexapi PUBLIC ${GSTREAMER_INCLUDE_DIRS})
target_include_directories(pyrhexapi PUBLIC $ENV{RHEX_API_DIR}/include)

# Link libraries
target_link_libraries(pyrhexapi PUBLIC
  $ENV{RHEX_API_DIR}/lib/librhexapi.a
  ssl
  crypto
  argon2
  atomic
  pthread
  gstreamer-1.0
  gstrtp-1.0
  gstvideo-1.0
  gobject-2.0
  ${GLIB_LIBRARIES})

cmake_minimum_required(VERSION 3.18)

# Vcpkg as a submodule
set(VCPKG_TARGET_TRIPLET "x64-windows-static")
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/vcpkg/scripts/buildsystems/vcpkg.cmake CACHE STRING "Vcpkg toolchain file")

project(Audio
  VERSION 0.1.0
  DESCRIPTION "Audio in C++"
  HOMEPAGE_URL "https://github.com/Haellsigh/Audio"
  LANGUAGES CXX)

# Qt MOC/UIC/RCC
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(Audio "")
target_sources(Audio PRIVATE
  main.cc
  mainwindow.cc
  mainwindow.hh
  mainwindow.ui
  fft.hh
  fft.cc
  audioio.hh
  audioio.cc
  timedelay.hh
  timedelay.cc
)

add_subdirectory(interface)
add_subdirectory(generators)
add_subdirectory(utilities)

##### Compilation options
# C++20
target_compile_features(Audio PUBLIC cxx_std_20)
set_target_properties(Audio PROPERTIES CXX_EXTENSIONS OFF)
if (MSVC)
  target_compile_options(Audio PUBLIC /Zc:__cplusplus)
endif()
# Enable Link Time Optimization if available
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported)
if(ipo_supported)
  set_target_properties(Audio PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
  message(STATUS "Enabled Link Time optimization")
endif()
# Warnings
if(MSVC)
  target_compile_options(Audio PRIVATE /W4)
else()
  target_compile_options(Audio PRIVATE -Wall -Wextra -Wconversion -Wsign-conversion)
endif()
# CCache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Enabled CCache")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()
# MT library
set_property(TARGET Audio PROPERTY
             MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

##### Libraries
# Qt
find_package(Qt5 COMPONENTS Widgets Multimedia PrintSupport REQUIRED)
target_link_libraries(Audio PRIVATE Qt5::Widgets Qt5::Multimedia Qt5::PrintSupport)

# Thirdparty libraries
add_subdirectory(thirdparty)

# FFTW
find_package(FFTW3 CONFIG REQUIRED)
find_package(FFTW3f CONFIG REQUIRED)
find_package(FFTW3l CONFIG REQUIRED)
target_link_libraries(Audio PRIVATE FFTW3::fftw3 FFTW3::fftw3f FFTW3::fftw3l)

# Ranges-v3
find_package(range-v3 CONFIG REQUIRED)
target_link_libraries(Audio PRIVATE range-v3 range-v3-meta range-v3::meta range-v3-concepts)

# Spdlog
find_package(spdlog CONFIG REQUIRED)
target_link_libraries(Audio PRIVATE spdlog::spdlog spdlog::spdlog_header_only)

# PortAudio
find_package(portaudio CONFIG REQUIRED)
target_link_libraries(Audio PRIVATE portaudio)

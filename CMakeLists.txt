cmake_minimum_required(VERSION 3.17)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake
  CACHE STRING "Vcpkg toolchain file")

message(STATUS "CMake ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" )


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
)

##### Compilation options
# C++20
target_compile_features(Audio PUBLIC cxx_std_20)
set_target_properties(Audio PROPERTIES CXX_EXTENSIONS OFF)
if (MSVC)
  target_compile_options(Audio PUBLIC /std:c++latest /Zc:__cplusplus)
endif()
# Enable Link Time Optimization if available
include(CheckIPOSupported)
check_ipo_supported(RESULT result)
if(result)
  set_target_properties(Audio PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
  message(STATUS "Enabled Link Time optimization")
endif()
# Warnings
if(MSVC)
  target_compile_options(Audio PRIVATE /W4)
else()
  target_compile_options(Audio PRIVATE -Wall -Wextra)
endif()

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Enabled CCache")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}") # CMake 3.9+
endif()

##### Libraries
# Qt
find_package(Qt5 COMPONENTS Core Widgets Multimedia PrintSupport REQUIRED)
target_link_libraries(Audio PRIVATE Qt5::Widgets Qt5::Multimedia Qt5::PrintSupport)

# FFTW
find_package(FFTW3 CONFIG REQUIRED)
target_link_libraries(Audio PRIVATE FFTW3::fftw3)
find_package(FFTW3f CONFIG REQUIRED)
target_link_libraries(Audio PRIVATE FFTW3::fftw3f)
find_package(FFTW3l CONFIG REQUIRED)
target_link_libraries(Audio PRIVATE FFTW3::fftw3l)

# Thirdparty libraries
add_subdirectory(thirdparty)

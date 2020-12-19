cmake_minimum_required(VERSION 3.18)

project(QtAwesome LANGUAGES CXX)

##### Qt MOC/RCC
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Library options
option(FontAwesomePro "Enable the pro icons" OFF)

##### Library
add_library(QtAwesome STATIC "")
target_sources(QtAwesome PRIVATE
    QtAwesome.h
    QtAwesome.cpp
    QtAwesomeAnim.h
    QtAwesomeAnim.cpp
)

target_include_directories(QtAwesome PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(FontAwesomePro)
    target_sources(QtAwesome PRIVATE QtAwesomePro.qrc)
    target_compile_definitions(QtAwesome PUBLIC FONT_AWESOME_PRO)
    message(STATUS "Using FontAwesome Pro")
else()
    target_sources(QtAwesome PRIVATE QtAwesomeFree.qrc)
    message(STATUS "Using FontAwesome Free")
endif()

##### Compilation options
# C++14
target_compile_features(QtAwesome PRIVATE cxx_std_14)
set_target_properties(QtAwesome PROPERTIES CXX_EXTENSIONS OFF)
if (MSVC)
    target_compile_options(QtAwesome PRIVATE /std:c++latest /Zc:__cplusplus)
endif()
# Enable Link Time Optimization if available
include(CheckIPOSupported)
check_ipo_supported(RESULT result)
if(result)
    set_target_properties(QtAwesome PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()
# Warnings
if(MSVC)
    target_compile_options(QtAwesome PRIVATE /W4)
else()
    target_compile_options(QtAwesome PRIVATE -Wall -Wextra -Wconversion -Wsign-conversion)
endif()
# CCache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Enabled CCache")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()
# MT library
set_property(TARGET QtAwesome PROPERTY
             MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

##### Qt
find_package(Qt5 COMPONENTS Widgets REQUIRED)
target_link_libraries(QtAwesome PRIVATE Qt5::Widgets)

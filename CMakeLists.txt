cmake_minimum_required(VERSION 3.30)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
include(toolchain)

##### Project
project(Audio
    VERSION 0.1.0
    DESCRIPTION "Audio in C++"
    HOMEPAGE_URL "https://github.com/Haellsigh/Audio"
    LANGUAGES CXX
)

##### Qt MOC/UIC/RCC
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
list(APPEND CMAKE_PREFIX_PATH "C:/Programs/Qt/6.7.2/msvc2019_64")
find_package(Qt6 COMPONENTS Core Widgets PrintSupport REQUIRED)
find_program(windeployqt windeployqt REQUIRED)

##### Executable
qt_add_executable(Audio MANUAL_FINALIZATION)
add_subdirectory(src)

##### Compilation options
target_set_default_options(Audio PRIVATE)
target_set_default_warnings(Audio PRIVATE)
target_set_warnings_as_error(Audio PRIVATE)
# CCache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Enabled CCache")
endif()

##### Libraries
find_package(FFTW3 CONFIG REQUIRED)
find_package(FFTW3f CONFIG REQUIRED)
find_package(FFTW3l CONFIG REQUIRED)
find_package(KFR CONFIG REQUIRED)
find_package(range-v3 CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(magic_enum CONFIG REQUIRED)
find_package(RtAudio CONFIG REQUIRED)

add_subdirectory(external)

target_link_libraries(Audio PRIVATE 
    FFTW3::fftw3 FFTW3::fftw3f FFTW3::fftw3l
    kfr kfr_dft
    Qt6::Core Qt6::Widgets QCustomPlot QtAwesome
    range-v3 range-v3-meta range-v3::meta range-v3-concepts
    fmt::fmt
    spdlog::spdlog
    magic_enum::magic_enum
    RtAudio::rtaudio
)

##### Qt dlls
add_custom_command(TARGET Audio POST_BUILD
    COMMENT "Deploying Qt dependencies"
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying Qt dependencies..."
    COMMAND ${windeployqt} $<TARGET_FILE:Audio> --no-translations
            --no-opengl-sw --no-system-d3d-compiler --no-quick-import)

##### Resources
add_custom_command(TARGET Audio POST_BUILD
    COMMENT "Deploying assets"
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying assets from ${CMAKE_CURRENT_SOURCE_DIR}/assets to $<TARGET_FILE_DIR:Audio>/assets"
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
    ${CMAKE_CURRENT_SOURCE_DIR}/assets $<TARGET_FILE_DIR:Audio>/assets
)

qt_finalize_executable(Audio)

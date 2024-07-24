cmake_minimum_required(VERSION 3.30)

include(target_set_default_options)
include(targets_warnings)

set(VCPKG_TARGET_TRIPLET x64-windows)
set(VCPKG_HOST_TRIPLET x64-windows)
set(QT_CREATOR_SKIP_VCPKG_SETUP ON)

include(${CMAKE_CURRENT_LIST_DIR}/../external/vcpkg/scripts/buildsystems/vcpkg.cmake)

set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

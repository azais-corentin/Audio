include(vcpkg/scripts/buildsystems/vcpkg.cmake)
include(vcpkg/scripts/cmake/vcpkg_configure_qmake.cmake)

target_sources(Audio PRIVATE
    qcustomplot/qcustomplot.h
    qcustomplot/qcustomplot.cpp
)

target_include_directories(Audio PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(FontAwesomePro ON)
add_subdirectory(QtAwesome)
target_link_libraries(Audio PRIVATE QtAwesome)

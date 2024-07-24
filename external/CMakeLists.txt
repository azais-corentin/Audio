cmake_minimum_required(VERSION 3.30)

# QCustomPlot
add_library(QCustomPlot )
target_sources(QCustomPlot PRIVATE
    qcustomplot/qcustomplot.h
    qcustomplot/qcustomplot.cpp
)

target_link_libraries(QCustomPlot PUBLIC
    Qt6::Core Qt6::Widgets Qt6::PrintSupport
)

target_include_directories(QCustomPlot PUBLIC SYSTEM
    ${CMAKE_CURRENT_SOURCE_DIR}/qcustomplot
)

add_subdirectory(QtAwesome)

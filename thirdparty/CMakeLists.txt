target_sources(Audio PRIVATE
  qcustomplot/qcustomplot.h
  qcustomplot/qcustomplot.cpp
)

target_include_directories(Audio PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

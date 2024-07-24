cmake_minimum_required(VERSION 3.30)

# Sources
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
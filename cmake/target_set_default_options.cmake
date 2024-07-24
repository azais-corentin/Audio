cmake_minimum_required(VERSION 3.30)

include(CheckIPOSupported)
check_ipo_supported(RESULT ipoSupported LANGUAGES ASM C CXX)

function(target_set_default_options Target Visibility)
    # Modern C++
    target_compile_features(${Target} ${Visibility} cxx_std_23)
    set_target_properties(${Target} PROPERTIES CXX_EXTENSIONS OFF)

    # Setup LTO
    if(ipoSupported)
        set_target_properties(${Target} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
    endif()
endfunction()
cmake_minimum_required(VERSION 3.30)

function(target_set_default_warnings Target Visibility)
    target_compile_options(${Target} ${Visibility} /W4)
endfunction()

function(target_set_warnings_as_error Target Visibility)
    target_compile_options(${Target} ${Visibility} /WX)
endfunction()
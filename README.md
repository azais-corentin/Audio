# Audio
Audio tests in C++

#### Setup

```
./thirdparty/vcpkg/bootstrap-vcpkg.bat -disableMetrics
./thirdparty/vcpkg/vcpkg install fftw3[core,avx,avx2,sse,sse2,threads] range-v3 portaudio
./thirdparty/vcpkg/vcpkg install --head spdlog
```

If portaudio fails to install, go to `thirdparty/vcpkg/ports/portaudio/portfile.cmake` 
and remove `PREFER_NINJA` from the vcpkg_configure_cmake function.

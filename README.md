# Audio
Audio tests in C++

#### Setup

```
./vcpkg/bootstrap-vcpkg.bat -disableMetrics
./vcpkg/vcpkg install fftw3[core,avx,avx2,sse,sse2,threads] range-v3 portaudio
./vcpkg/vcpkg install --head spdlog
```

If portaudio fails to install, go to `./vcpkg/ports/portaudio/portfile.cmake` 
and remove `PREFER_NINJA` from the vcpkg_configure_cmake function.

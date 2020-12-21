#pragma once

#include <complex>
#include <vector>

namespace Audio {

class fft {
    using fcomplex = std::complex<float>;

  public:
    enum class Direction
    {
        Forward  = -1,
        Backward = +1
    };

  public:
    // Forward
    static std::vector<fcomplex> r2c(std::vector<float> &input);

    // Backward
    static std::vector<float> c2r(std::vector<fcomplex> &input);

    // Forward/Backward
    static std::vector<fcomplex> c2c(std::vector<fcomplex> &input, Direction d);
    static std::vector<float> r2r(std::vector<float> &input, Direction d);
};

} // namespace Audio

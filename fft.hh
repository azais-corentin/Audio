#pragma once

#include <complex>
#include <vector>

namespace Audio {

class fft {
  using fcomplex = std::complex<float>;

 public:
  enum class Direction { Forward = -1, Backward = +1 };

 public:
  // Forward
  static std::vector<fcomplex> r2c(std::vector<float>& input);

  // Backward
  static std::vector<float> c2r(std::vector<fcomplex>& input);

  static std::vector<fcomplex> c2c(std::vector<fcomplex>& input, Direction d);
};

}  // namespace Audio

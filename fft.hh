#pragma once

#include <complex>
#include <vector>

class fft {
  using fcomplex = std::complex<float>;

 public:
  // Forward
  static std::vector<fcomplex> r2c(std::vector<float>& input);

  // Backward
  static std::vector<float> c2r(std::vector<fcomplex>& input);

  // Data extraction
};

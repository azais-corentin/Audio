#pragma once

#include <complex>
#include <vector>

class fft {
  using complex = std::complex<double>;

 public:
  // Forward
  static std::vector<complex> r2c(std::vector<double>& input);

  // Backward
  static std::vector<double> c2r(std::vector<complex>& input);

  // Data extraction
};

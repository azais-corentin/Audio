#pragma once

#include <complex>
#include <vector>

class fft {
  using complex = std::complex<double>;

 public:
  // FFT functions
  static std::vector<complex> r2c(std::vector<double> input);

  // Data extraction
};

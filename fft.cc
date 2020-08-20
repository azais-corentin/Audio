#include "fft.hh"

#include <fftw3.h>

std::vector<fft::complex> fft::r2c(std::vector<double> input)
{
  const auto n    = input.size();
  const auto nFFT = n / 2 + 1;

  std::vector<fft::complex> output;
  output.reserve(nFFT);
  output.resize(nFFT);
  fftw_plan p = fftw_plan_dft_r2c_1d(
      n, input.data(), reinterpret_cast<fftw_complex*>(output.data()), FFTW_ESTIMATE);
  fftw_execute(p);
  fftw_destroy_plan(p);
  return output;
}

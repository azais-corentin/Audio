#include "fft.hh"

#include <fftw3.h>

std::vector<fft::complex> fft::r2c(std::vector<double>& input)
{
  const auto nInput  = input.size();
  const auto nOutput = nInput / 2;

  std::vector<fft::complex> output;
  output.reserve(nOutput);
  output.resize(nOutput);
  fftw_plan p = fftw_plan_dft_r2c_1d(nInput, input.data(),
                                     reinterpret_cast<fftw_complex*>(output.data()), FFTW_ESTIMATE);
  fftw_execute(p);
  fftw_destroy_plan(p);
  return output;
}

std::vector<double> fft::c2r(std::vector<fft::complex>& input)
{
  const auto nInput  = input.size();
  const auto nOutput = 2 * (nInput - 1);

  std::vector<double> output;
  output.reserve(nOutput);
  output.resize(nOutput);
  fftw_plan p = fftw_plan_dft_c2r_1d(nInput, reinterpret_cast<fftw_complex*>(input.data()),
                                     output.data(), FFTW_ESTIMATE);
  fftw_execute(p);
  fftw_destroy_plan(p);
  return output;
}

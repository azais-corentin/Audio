#include "fft.hh"

#include <fftw3.h>

std::vector<fft::fcomplex> fft::r2c(std::vector<float>& input)
{
  const auto nInput  = input.size();
  const auto nOutput = nInput / 2;

  std::vector<fft::fcomplex> output;
  output.reserve(nOutput);
  output.resize(nOutput);
  auto p = fftwf_plan_dft_r2c_1d(nInput, input.data(),
                                 reinterpret_cast<fftwf_complex*>(output.data()), FFTW_ESTIMATE);
  fftwf_execute(p);
  fftwf_destroy_plan(p);
  return output;
}

std::vector<float> fft::c2r(std::vector<fft::fcomplex>& input)
{
  const auto nInput  = input.size();
  const auto nOutput = 2 * (nInput - 1);

  std::vector<float> output;
  output.reserve(nOutput);
  output.resize(nOutput);
  auto p = fftwf_plan_dft_c2r_1d(nInput, reinterpret_cast<fftwf_complex*>(input.data()),
                                 output.data(), FFTW_ESTIMATE);
  fftwf_execute(p);
  fftwf_destroy_plan(p);
  return output;
}

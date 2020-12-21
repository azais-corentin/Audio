#include "fft.hh"

#include <fftw3.h>

namespace Audio {

std::vector<fft::fcomplex> fft::r2c(std::vector<float> &input) {
    const auto nInput  = input.size();
    const auto nOutput = nInput / 2 + 1;

    std::vector<fft::fcomplex> output;
    output.reserve(nOutput);
    output.resize(nOutput);
    auto p =
        fftwf_plan_dft_r2c_1d(static_cast<int>(nInput), input.data(), reinterpret_cast<fftwf_complex *>(output.data()),
                              FFTW_ESTIMATE | FFTW_PRESERVE_INPUT);
    fftwf_execute(p);
    fftwf_destroy_plan(p);
    return output;
}

std::vector<float> fft::c2r(std::vector<fft::fcomplex> &input) {
    const auto nInput  = input.size();
    const auto nOutput = 2 * (nInput - 1);

    std::vector<float> output;
    output.reserve(nOutput);
    output.resize(nOutput);
    auto p = fftwf_plan_dft_c2r_1d(static_cast<int>(nInput), reinterpret_cast<fftwf_complex *>(input.data()),
                                   output.data(), FFTW_ESTIMATE | FFTW_PRESERVE_INPUT);
    fftwf_execute(p);
    fftwf_destroy_plan(p);
    return output;
}

std::vector<fft::fcomplex> fft::c2c(std::vector<fft::fcomplex> &input, Direction d) {
    const auto nInput = input.size();

    std::vector<fft::fcomplex> output(nInput);
    output.resize(nInput);

    auto p = fftwf_plan_dft_1d(static_cast<int>(nInput), reinterpret_cast<fftwf_complex *>(input.data()),
                               reinterpret_cast<fftwf_complex *>(output.data()), static_cast<int>(d),
                               FFTW_ESTIMATE | FFTW_PRESERVE_INPUT);

    fftwf_execute(p);
    fftwf_destroy_plan(p);
    return output;
}

std::vector<float> fft::r2r(std::vector<float> &input) {
    const auto n = input.size();
    std::vector<float> output(n);
    output.resize(n);

    auto p = fftwf_plan_r2r_1d(n, reinterpret_cast<float *>(input.data()), reinterpret_cast<float *>(output.data()),
                               FFTW_REDFT00, FFTW_ESTIMATE);

    fftwf_execute(p);
    fftwf_destroy_plan(p);

    return output;
}

} // namespace Audio

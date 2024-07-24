#include "fft.hh"

#include <fftw3.h>

namespace Audio::Fft
{

std::vector<ComplexFloat> r2c(std::span<const float> input)
{
    const auto nInput  = input.size();
    const auto nOutput = nInput / 2 + 1;

    std::vector<ComplexFloat> output;
    output.reserve(nOutput);
    output.resize(nOutput);

    auto *plan = fftwf_plan_dft_r2c_1d(
        static_cast<int>(nInput), const_cast<float *>(input.data()),
        reinterpret_cast<fftwf_complex *>(output.data()), FFTW_ESTIMATE | FFTW_PRESERVE_INPUT
    );
    fftwf_execute(plan);
    fftwf_destroy_plan(plan);
    return output;
}

std::vector<float> c2r(std::span<const ComplexFloat> input)
{
    const auto nInput  = input.size();
    const auto nOutput = 2 * (nInput - 1);

    std::vector<float> output;
    output.reserve(nOutput);
    output.resize(nOutput);

    auto *plan = fftwf_plan_dft_c2r_1d(
        static_cast<int>(nInput),
        reinterpret_cast<fftwf_complex *>(const_cast<ComplexFloat *>(input.data())), output.data(),
        FFTW_ESTIMATE | FFTW_PRESERVE_INPUT
    );
    fftwf_execute(plan);
    fftwf_destroy_plan(plan);
    return output;
}

std::vector<ComplexFloat> c2c(std::span<const ComplexFloat> input, Direction direction)
{
    const auto nInput = input.size();

    std::vector<ComplexFloat> output(nInput);
    output.resize(nInput);

    auto *plan = fftwf_plan_dft_1d(
        static_cast<int>(nInput),
        reinterpret_cast<fftwf_complex *>(const_cast<ComplexFloat *>(input.data())),
        reinterpret_cast<fftwf_complex *>(output.data()), static_cast<int>(direction),
        FFTW_ESTIMATE | FFTW_PRESERVE_INPUT
    );

    fftwf_execute(plan);
    fftwf_destroy_plan(plan);
    return output;
}

std::vector<float> r2r(std::span<const float> input)
{
    const auto n = input.size();
    std::vector<float> output(n);
    output.resize(n);

    auto p = fftwf_plan_r2r_1d(
        static_cast<int>(n), reinterpret_cast<float *>(const_cast<float *>(input.data())),
        reinterpret_cast<float *>(output.data()), FFTW_REDFT00, FFTW_ESTIMATE
    );

    fftwf_execute(p);
    fftwf_destroy_plan(p);

    return output;
}

} // namespace Audio::Fft

#pragma once

#include <complex>
#include <span>
#include <vector>

namespace Audio::Fft
{

using ComplexFloat = std::complex<float>;

enum class Direction { Forward = -1, Backward = +1 };

// Forward
std::vector<ComplexFloat> r2c(std::span<const float> input);

// Backward
std::vector<float> c2r(std::span<const ComplexFloat> input);

// Forward/Backward
std::vector<ComplexFloat> c2c(std::span<const ComplexFloat> input, Direction direction);
std::vector<float> r2r(std::span<const float> input);

} // namespace Audio::Fft

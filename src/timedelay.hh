#pragma once

#include <span>
#include <vector>

namespace Audio::TimeDelay {

enum Estimator
{
    CrossCorrelation,
    PhaseDifference /*, AverageSquareDifference*/
};

std::size_t estimate(
    std::span<const float> reference, std::span<const float> measured, uint32_t max_delay,
    Estimator method
);

std::vector<float> cc(std::span<const float> a, std::span<const float> b, uint32_t max_delay);
std::vector<float> phat(std::span<const float> a, std::span<const float> b, uint32_t max_delay);
// fvec asdf(const fvec& a, const fvec& b, std::size_t max_delay);

} // namespace Audio::TimeDelay

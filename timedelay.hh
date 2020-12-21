#pragma once

#include <complex>
#include <vector>

namespace Audio::TimeDelay {

enum Estimator
{
    CrossCorrelation,
    PhaseDifference /*, AverageSquareDifference*/
};

std::size_t estimate(std::vector<float> &reference, std::vector<float> &measured, std::size_t max_delay,
                     Estimator method);

std::vector<float> cc(std::vector<float> &a, std::vector<float> &b, std::size_t max_delay);
std::vector<float> phat(std::vector<float> &a, std::vector<float> &b, std::size_t max_delay);
// fvec asdf(const fvec& a, const fvec& b, std::size_t max_delay);

} // namespace Audio::TimeDelay

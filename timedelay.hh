#pragma once

#include <vector>

namespace TimeDelay {
enum Estimator {
  CrossCorrelation,
  PhaseDifference /*, AverageSquareDifference*/
};

int32_t estimate(const std::vector<float>& a,
                 const std::vector<float>& b,
                 std::size_t               max_delay,
                 Estimator                 method);

std::vector<float> cc(const std::vector<float>& a,
                      const std::vector<float>& b,
                      std::size_t               max_delay);
std::vector<float> phat(const std::vector<float>& a,
                        const std::vector<float>& b,
                        std::size_t               max_delay);
// fvec asdf(const fvec& a, const fvec& b, std::size_t max_delay);

}  // namespace TimeDelay

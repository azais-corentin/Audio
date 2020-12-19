#include "timedelay.hh"
#include "fft.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <execution>
#include <numeric>

namespace Audio::TimeDelay {

std::size_t estimate(std::vector<float>& a,
                     std::vector<float>& b,
                     std::size_t         max_delay,
                     Estimator           method)
{
  std::vector<float> values;
  switch (method) {
    case Estimator::CrossCorrelation:
      values = cc(a, b, max_delay);
      break;
    case Estimator::PhaseDifference:
      values = phat(a, b, max_delay);
      break;
  }

  auto max = std::max_element(values.begin(), values.end());
  return std::distance(values.begin(), max) - max_delay;
}

std::vector<float> cc(std::vector<float>& a, std::vector<float>& b, std::size_t max_delay)
{
  std::vector<float>   values(2 * max_delay + 1);
  std::vector<int32_t> delays(values.size());
  std::iota(delays.begin(), delays.end(), -static_cast<int32_t>(max_delay));

  std::transform(std::execution::par, delays.begin(), delays.end(), values.begin(),
                 [&](int32_t delay) {
                   if (delay < 0) {
                     return std::transform_reduce(std::execution::par_unseq,
                                                  std::next(a.begin(), -delay), a.end(),
                                                  b.begin(), 0.f);
                   } else {
                     return std::transform_reduce(std::execution::par_unseq, a.begin(),
                                                  std::next(a.end(), -delay),
                                                  std::next(b.begin(), delay), 0.f);
                   }
                 });

  return values;
}

std::vector<float> phat(std::vector<float>& a, std::vector<float>& b, std::size_t)
{
  std::vector<std::complex<float>> ac(a.size());
  std::vector<std::complex<float>> bc(b.size());
  std::transform(std::execution::par_unseq, a.begin(), a.end(), ac.begin(),
                 [](const auto& a) { return std::complex(a); });
  std::transform(std::execution::par_unseq, b.begin(), b.end(), bc.begin(),
                 [](const auto& b) { return std::complex(b); });

  auto                             a_dft = fft::c2c(ac, fft::Direction::Forward);
  auto                             b_dft = fft::c2c(bc, fft::Direction::Forward);
  std::vector<std::complex<float>> g_phat(a_dft.size());

  std::transform(std::execution::seq, a_dft.begin(), a_dft.end(), b_dft.begin(),
                 g_phat.begin(),
                 [](const std::complex<float>& a, const std::complex<float>& b) {
                   const auto abconj = a * std::conj(b);
                   return abconj / std::abs(abconj);
                 });

  spdlog::info("a::size {}", a.size());
  spdlog::info("a_dft::size {}", a_dft.size());
  spdlog::info("g_phat::size {}", g_phat.size());

  return fft::c2r(g_phat);
  /*
  auto values = cc(a, b, max_delay);

  spdlog::info("values::size {}", values.size());

  auto values_dft = fft::r2c(values);

  spdlog::info("values_dft::size {}", values_dft.size());

  std::transform(
      std::begin(values_dft), std::end(values_dft), std::begin(values_dft),
      [](const std::complex<float>& value) { return value / std::abs(value); });
  auto values_phat = fft::c2r(values_dft);

  spdlog::info("values_phat::size {}", values_phat.size());

  std::transform(std::begin(values_phat), std::end(values_phat), std::begin(values_phat),
                 [](const float& value) { return std::abs(value); });

  return values_phat;
*/
}

}  // namespace Audio::TimeDelay

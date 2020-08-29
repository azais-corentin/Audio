#include "timedelay.hh"
#include "fft.hh"

#include <algorithm>
#include <execution>
#include <numeric>

template <typename ContainerT>
auto bounded_next(const ContainerT&                   container,
                  typename ContainerT::const_iterator it,
                  typename std::iterator_traits<
                      typename ContainerT::const_iterator>::difference_type n = 1)
{
  return std::clamp(std::next(it, n), std::cbegin(container),
                    std::prev(std::cend(container)));
}

namespace TimeDelay {

int32_t estimate(const std::vector<float>& a,
                 const std::vector<float>& b,
                 std::size_t               max_delay,
                 Estimator                 method)
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

std::vector<float> cc(const std::vector<float>& a,
                      const std::vector<float>& b,
                      std::size_t               max_delay)
{
  std::vector<float>   values(2 * max_delay + 1);
  std::vector<int32_t> delays(values.size());
  std::iota(delays.begin(), delays.end(), -max_delay);

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

                   return std::transform_reduce(std::execution::par_unseq,
                                                bounded_next(a, a.begin(), -delay),
                                                bounded_next(a, a.end(), -delay),
                                                bounded_next(b, b.begin(), delay), 0.f);
                 });

  return values;
}

std::vector<float> phat(const std::vector<float>& a,
                        const std::vector<float>& b,
                        std::size_t               max_delay)
{
  auto values = cc(a, b, max_delay);

  auto values_dft = fft::r2c(values);
  std::transform(
      std::begin(values_dft), std::end(values_dft), std::begin(values_dft),
      [](const std::complex<float>& value) { return value / std::abs(value); });
  auto values_phat = fft::c2r(values_dft);

  std::transform(std::begin(values_phat), std::end(values_phat), std::begin(values_phat),
                 [](const float& value) { return std::abs(value); });

  return values_phat;
}

}  // namespace TimeDelay

#include "timedelay.hh"

#include "fft.hh"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdint>
#include <execution>
#include <numeric>

namespace Audio::TimeDelay {

/**
 * @brief Estimates the delay (in samples) of measured with regards to reference.
 *
 * @note Assumes that the delay is positive.
 */
std::size_t estimate(
    std::span<const float> reference, std::span<const float> measured, uint32_t max_delay,
    Estimator method
)
{
    switch (method) {
    case Estimator::CrossCorrelation: {
        const auto values = cc(reference, measured, max_delay);
        // Find the maximum absolute value
        const auto [argmin, argmax] = std::minmax_element(values.begin(), values.end());
        const auto delay_samples    = std::abs(*argmax) > std::abs(*argmin) ? argmax : argmin;
        return static_cast<std::size_t>(std::distance(values.begin(), delay_samples));
    }
    case Estimator::PhaseDifference: {
        const auto values = phat(reference, measured, max_delay);
        // Find the maximum absolute value
        const auto [argmin, argmax] = std::minmax_element(values.begin(), values.end());
        const auto delay_samples    = std::abs(*argmax) > std::abs(*argmin) ? argmax : argmin;
        const auto final_value =
            2 * static_cast<int64_t>(std::distance(delay_samples, values.end())) - static_cast<int64_t>(values.size());
        if (final_value < 0) { spdlog::error("This shouldn't be negative: ", final_value); }
        return static_cast<std::size_t>(final_value);
    }
    default:
        spdlog::error("Unhandled switch case");
        assert(false);
        break;
    }

    return 0;
}

std::vector<float> cc(std::span<const float> a, std::span<const float> b, uint32_t max_delay)
{
    std::size_t count = max_delay + 1;
    std::vector<float> values(count);
    std::vector<int32_t> delays(count);
    std::iota(delays.begin(), delays.end(), 0);

    std::transform(std::execution::par_unseq, delays.begin(), delays.end(), values.begin(), [&](int32_t delay) {
        return std::transform_reduce(std::execution::par_unseq, a.begin(), std::next(a.end(), -delay),
                                     std::next(b.begin(), delay), 0.f);
    });

    return values;
}

std::vector<float> phat(std::span<const float> a, std::span<const float> b, uint32_t /*max_delay*/)
{
    /*std::vector<std::complex<float>> ac(a.size());
    std::vector<std::complex<float>> bc(b.size());
    std::transform(std::execution::par_unseq, a.begin(), a.end(), ac.begin(),
                   [](const auto &a) { return std::complex(a); });
    std::transform(std::execution::par_unseq, b.begin(), b.end(), bc.begin(),
                   [](const auto &b) { return std::complex(b); });*/

    auto a_dft = Fft::r2c(a);
    auto b_dft = Fft::r2c(b);
    std::vector<std::complex<float>> g_phat(a_dft.size());

    assert(a_dft.size() == b_dft.size());

    std::transform(std::execution::seq, a_dft.begin(), a_dft.end(), b_dft.begin(), g_phat.begin(),
                   [](const std::complex<float> &a, const std::complex<float> &b) { return a * std::conj(b); });

    spdlog::info("a::size {}", a.size());
    spdlog::info("a_dft::size {}", a_dft.size());
    spdlog::info("g_phat::size {}", g_phat.size());

    return Fft::c2r(g_phat);
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

} // namespace Audio::TimeDelay

#include "synchronizedsweptsine.hh"

#include <cmath>
#include <numbers>

namespace Audio::Generator {

SynchronizedSweptSine::SynchronizedSweptSine(float f1, float f2, std::size_t length, std::size_t sample_rate)
    : BaseInterface<SynchronizedSweptSine>(length, sample_rate), m_f1(f1), m_f2(f2),
      m_L(std::round(f1 * duration() / std::log(f2 / f1)) / f1) {
    m_length = static_cast<std::size_t>(m_L * std::log(f2 / f1) * sample_rate);
}

float SynchronizedSweptSine::sample(std::size_t index) const {
    if (!finished(index)) return std::sin(2.f * std::numbers::pi_v<float> * m_f1 * m_L * std::exp(time(index) / m_L));

    return 0;
}

std::vector<std::complex<float>> SynchronizedSweptSine::xtilde() const {
    using namespace std::complex_literals;
    const std::size_t n = m_length / 2 + 1;
    std::vector<std::complex<float>> output(n);

    std::generate(output.begin(), output.end(), [&, k = 0]() mutable {
        const float f = k++ * 2.f * m_sample_rate / 2.f / n;
        return 2.f * std::sqrt(f / m_L) *
               std::exp(-2if * std::numbers::pi_v<float> * f * m_L * (1 - std::log(f / m_f1)) +
                        1if * std::numbers::pi_v<float> / 4.f);
    });

    return output;
}

} // namespace Audio::Generator

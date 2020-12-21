#include "synchronizedsweptsine.hh"

#include <cmath>
#include <numbers>

namespace Audio::Generator {

SynchronizedSweptSine::SynchronizedSweptSine(float f1, float f2, std::size_t length, std::size_t sample_rate)
    : BaseInterface<SynchronizedSweptSine>(length, sample_rate), f1_(f1), f2_(f2),
      k_(std::round(duration() * f1 / std::log(f2 / f1))), L_(k_ / f1) {
    // Compute a new length from k, f1, f2 and the sample rate
    length_ = static_cast<std::size_t>(sample_rate_ * k_ * std::log(f2_ / f1_) / f1_);
}

float SynchronizedSweptSine::sample(std::size_t index) const {
    if (!finished(index)) { return std::sin(2.f * std::numbers::pi_v<float> * f1_ * L_ * std::exp(time(index) / L_)); }

    return 0;
}

// std::vector<std::complex<float>> SynchronizedSweptSine::xtilde() const {
//    using namespace std::complex_literals;
//    const std::size_t n = m_length / 2 + 1;
//    std::vector<std::complex<float>> output(n);

//    std::generate(output.begin(), output.end(), [&, k = 0]() mutable {
//        const float f = k++ * 2.f * m_sample_rate / 2.f / n;
//        return 2.f * std::sqrt(f / m_L) *
//               std::exp(-2if * std::numbers::pi_v<float> * f * m_L * (1 - std::log(f / m_f1)) +
//                        1if * std::numbers::pi_v<float> / 4.f);
//    });

//    return output;
//}

} // namespace Audio::Generator

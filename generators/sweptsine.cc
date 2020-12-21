#include "sweptsine.hh"

#include <cmath>
#include <numbers>

namespace Audio::Generator {

SweptSine::SweptSine(float f1, float f2, std::size_t length, std::size_t sample_rate)
    : BaseInterface<SweptSine>(length, sample_rate), m_f1(f1), m_f2(f2), m_ln(std::log(m_f2 / m_f1)),
      m_Tln(static_cast<float>(length) / static_cast<float>(sample_rate) / m_ln) {}

float SweptSine::sample(std::size_t index) const {
    if (!finished(index)) {
        return std::sin(2.f * std::numbers::pi_v<float> * m_f1 * m_Tln *
                        std::expm1(m_ln * static_cast<float>(index) / static_cast<float>(length_)));
    }

    return 0;
}

} // namespace Audio::Generator

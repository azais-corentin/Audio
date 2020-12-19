#pragma once

#include "baseinterface.hh"

#include <complex>

namespace Audio::Generator {

class SynchronizedSweptSine : public BaseInterface<SynchronizedSweptSine> {
  public:
    using BaseInterface::BaseInterface;
    SynchronizedSweptSine(float f1, float f2, std::size_t length, std::size_t sample_rate);

    float sample(std::size_t index) const;

    std::vector<std::complex<float>> xtilde() const;

  private:
    friend class BaseInterface<SynchronizedSweptSine>;
    // Configured parameters
    const float m_f1, m_f2;

    // Computed parameters
    const float m_L;
};

} // namespace Audio::Generator

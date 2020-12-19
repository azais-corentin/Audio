#pragma once

#include "baseinterface.hh"

namespace Audio::Generator {

class SweptSine : public BaseInterface<SweptSine> {
 public:
  using BaseInterface::BaseInterface;
  SweptSine(float f1, float f2, std::size_t length, std::size_t sample_rate);

  float sample(std::size_t index) const;

 private:
  friend class BaseInterface<SweptSine>;
  // Configured parameters
  const float m_f1, m_f2;

  // Computed parameters
  const float m_ln;
  const float m_Tln;
};

}  // namespace Audio::Generator

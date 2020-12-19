#pragma once

#include <spdlog/spdlog.h>

#include <cstddef>

namespace Audio::Generator {

template <class Derived>
class BaseInterface {
 public:
  inline float sample(std::size_t index) const { return static_base().sample(index); }
  inline bool  finished(std::size_t index) const { return index >= m_length; }

  std::size_t  length() const { return m_length; }
  std::size_t  sample_rate() const { return m_sample_rate; }
  inline float duration() const { return static_cast<float>(m_length) / m_sample_rate; }
  inline float time(std::size_t sample) const
  {
    return static_cast<float>(sample) / m_sample_rate;
  }

 private:
  friend Derived;
  BaseInterface() = default;
  BaseInterface(std::size_t length, std::size_t sample_rate)
      : m_length(length), m_sample_rate(sample_rate)
  {}

  inline Derived&       static_base() { return static_cast<Derived&>(*this); }
  inline Derived const& static_base() const { return static_cast<Derived const&>(*this); }

 protected:
  std::size_t m_length      = 0;
  std::size_t m_sample_rate = 0;
};

/*!
 * \brief Default empty generator.
 *
 * Generates no data.
 */
class Null : public BaseInterface<Null> {
 public:
  using BaseInterface::BaseInterface;
  Null();

  float sample(std::size_t) const;

 private:
  friend class BaseInterface<Null>;
};

}  // namespace Audio::Generator

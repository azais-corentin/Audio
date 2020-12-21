#pragma once

#include <spdlog/spdlog.h>

#include <cstddef>

namespace Audio::Generator {

/**
 * Base generator interface.
 *
 * Generates audio samples from a derived generator class.
 * Child classes should reimplement the `float sample(std::size_t index)` function and return a value for each `index`
 * from 0 to m_length - 1 included.
 *
 */
template <class Derived> class BaseInterface {
  public:
    inline float sample(std::size_t index) const { return static_base().sample(index); }
    inline bool finished(std::size_t index) const { return index >= length_; }

    std::size_t length() const { return length_; }
    std::size_t sample_rate() const { return sample_rate_; }
    inline float duration() const { return static_cast<float>(length_) / sample_rate_; }
    inline float time(std::size_t sample) const { return static_cast<float>(sample) / sample_rate_; }

  private:
    friend Derived;
    BaseInterface() = default;
    BaseInterface(std::size_t length, std::size_t sample_rate) : length_(length), sample_rate_(sample_rate) {}

    inline Derived &static_base() { return static_cast<Derived &>(*this); }
    inline Derived const &static_base() const { return static_cast<Derived const &>(*this); }

  protected:
    std::size_t length_      = 0;
    std::size_t sample_rate_ = 0;
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

} // namespace Audio::Generator

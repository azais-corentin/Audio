#pragma once

#include <spdlog/spdlog.h>

#include <cstdint>

namespace Audio::Generators {

/**
 * Base generator interface.
 *
 * Generates audio samples from a derived generator class.
 * Child classes should reimplement the `float sample(std::size_t index)` function and return a value for each `index`
 * from 0 to m_length - 1 included.
 *
 */
class AbstractGenerator
{
  public:
    AbstractGenerator() = delete;
    AbstractGenerator(const AbstractGenerator &)            = default;
    AbstractGenerator(AbstractGenerator &&)                 = delete;
    AbstractGenerator &operator=(const AbstractGenerator &) = delete;
    AbstractGenerator &operator=(AbstractGenerator &&)      = delete;
    virtual ~AbstractGenerator()                            = default;

    [[nodiscard]] virtual float sample(uint32_t index) const = 0;

    [[nodiscard]] bool finished(uint32_t index, bool includeSilence = true) const;
    [[nodiscard]] uint32_t length(bool includeSilence = true) const;
    [[nodiscard]] uint32_t sampleRate() const;
    [[nodiscard]] float duration(bool includeSilence = true) const;
    [[nodiscard]] float time(uint32_t index) const;
    [[nodiscard]] uint32_t index(float time) const;
    [[nodiscard]] float progress(uint32_t index, bool includeSilence = true) const;

  protected:
    AbstractGenerator(
        uint32_t length, uint32_t silenceLength, uint32_t sampleRate, float amplitude
    );

  protected:
    const uint32_t length_, silenceLength_;
    const uint32_t sampleRate_;
    const float amplitude_;
};

} // namespace Audio::Generators

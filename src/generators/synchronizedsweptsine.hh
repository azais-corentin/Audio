#pragma once

#include "abstractgenerator.hh"

namespace Audio::Generators {

class SynchronizedSweptSine : public AbstractGenerator
{
  public:
    SynchronizedSweptSine(
        float frequencyBegin, float frequencyEnd, uint32_t length, uint32_t silenceLength,
        uint32_t sampleRate, float amplitude
    );

    [[nodiscard]] float sample(uint32_t index) const final;

  private:
    const float frequencyBegin_, frequencyEnd_;
    const float sweepRate_, factor_;
};

} // namespace Audio::Generators

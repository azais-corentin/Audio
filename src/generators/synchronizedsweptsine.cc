#include "synchronizedsweptsine.hh"

#include <cmath>
#include <numbers>

namespace Audio::Generators
{

SynchronizedSweptSine::SynchronizedSweptSine(
    float frequencyBegin, float frequencyEnd, uint32_t length, uint32_t silenceLength,
    uint32_t sampleRate, float amplitude
)
    : AbstractGenerator{length, silenceLength, sampleRate, amplitude},
      frequencyBegin_{frequencyBegin}, frequencyEnd_{frequencyEnd},
      sweepRate_{std::log(frequencyEnd_ / frequencyBegin_)},
      factor_{2.F * std::numbers::pi_v<float> * frequencyBegin_ * duration() / sweepRate_}
{
}

float SynchronizedSweptSine::sample(uint32_t index) const
{
    if(finished(index, false)) [[unlikely]] { return 0; }

    return amplitude_ * std::sin(factor_ * std::exp(sweepRate_ * progress(index)));
}

} // namespace Audio::Generators

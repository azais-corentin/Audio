#include "abstractgenerator.hh"

namespace Audio::Generators
{

bool AbstractGenerator::finished(uint32_t index, bool includeSilence) const
{
    return index >= length(includeSilence);
}

uint32_t AbstractGenerator::length(bool includeSilence) const
{
    return length_ + (includeSilence ? silenceLength_ : 0);
}

uint32_t AbstractGenerator::sampleRate() const { return sampleRate_; }

float AbstractGenerator::duration(bool includeSilence) const
{
    return static_cast<float>(length(includeSilence)) / static_cast<float>(sampleRate_);
}

float AbstractGenerator::time(uint32_t index) const
{
    return static_cast<float>(index) / static_cast<float>(sampleRate_);
}

uint32_t AbstractGenerator::index(float time) const
{
    return static_cast<uint32_t>(time * static_cast<float>(sampleRate_));
}

float AbstractGenerator::progress(uint32_t index, bool includeSilence) const
{
    return static_cast<float>(index) / static_cast<float>(length(includeSilence));
}

AbstractGenerator::AbstractGenerator(
    uint32_t length, uint32_t silenceLength, uint32_t sampleRate, float amplitude
)
    : length_{length}, silenceLength_{silenceLength}, sampleRate_{sampleRate}, amplitude_{amplitude}
{
}

} // namespace Audio::Generators

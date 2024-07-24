#pragma once

#include "generators/generators.hh"

#include <QObject>

#include <array>
#include <chrono>
#include <memory>
#include <span>
#include <vector>

class RtAudio;

namespace Audio {

class AudioIO;

struct SweepParameters {
    float f0, ff;
    float amplitude;

    float k, log_k;
};

using Generator = std::unique_ptr<Generators::AbstractGenerator>;

struct SweepData {
    AudioIO *parent = nullptr;
    Generator generator; // Generates n samples
    uint32_t index = 0;
    uint32_t outputChannelCount, inputChannelCount; // Number of channels

    std::vector<float> measuredData;  // Measured by the microphone
    std::vector<float> referenceData; // Populated by the generator

    SweepData() = default;

    SweepData(
        AudioIO *parent_, Generator generator_, uint32_t outputChannelCount_,
        uint32_t inputChannelCount_
    )
        : parent{parent_}, generator{std::move(generator_)},
          outputChannelCount{outputChannelCount_}, inputChannelCount{inputChannelCount_}
    {
    }
};

struct AudioData2 {
    AudioIO *parent;
    std::size_t output_channels_count;
    std::size_t index;
    std::size_t silence_length;
    float amplitude;
    std::vector<float> data_measured;  ///< Measured signal, from the microphone
    std::vector<float> data_reference; ///< Reference signal, going to the speakers
    Generator generator;

    template<typename... Args>
    void init(
        AudioIO *parent_, std::size_t output_channels_count_, std::size_t silence_length_,
        float amplitude_, Generator generator_
    )
    {
        generator             = std::move(generator_);
        parent                = parent_;
        output_channels_count = output_channels_count_;
        index                 = 0;
        silence_length        = silence_length_;
        amplitude             = amplitude_;
        auto length           = std::visit([](const auto &g) { return g.length(); }, generator);
        data_measured.clear();
        data_measured.reserve(length + silence_length);
        data_reference.clear();
        data_reference.reserve(length + silence_length);
        spdlog::info("length + silence_length: {}", length + silence_length);
    }
};

struct RTAAudioData {
    AudioIO *parent_;
    bool stop_          = false;
    std::size_t buffer_ = 0, index_ = 0;
    std::array<float, 24000> data_;

    void init(AudioIO *parent) {
        parent_ = parent;
        stop_   = false;
        buffer_ = 0;
        index_  = 0;
        data_.fill(0);
    }
};

class AudioIO : public QObject {
    Q_OBJECT

  public:
    AudioIO();
    ~AudioIO();

    std::span<const uint32_t> supportedSampleRates();

    void startSweep(
        float frequencyBegin, float frequencyEnd, uint32_t length, uint32_t silenceLength,
        uint32_t sampleRate, float volumeDBFS
    );
    void stopSweep();
    SweepData const &getSweepData();

    // void startRTA();
    // bool rta_state();

    // void handle_rta_data(std::array<float, 24000> data);
    // std::array<float, 24000> get_latest_rta_data_();

  signals:
    void sweepFinished();
    // void on_rta_data();

  private:
    SweepData sweepData_;
    RTAAudioData rta_data_;
    std::chrono::high_resolution_clock::time_point timeStart_ = std::chrono::high_resolution_clock::now();
    std::unique_ptr<RtAudio> audio_;

    std::vector<uint32_t> supportedSampleRates_;
    std::array<float, 24'000> latestRTAData_;
};

} // namespace Audio

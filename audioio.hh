#pragma once

#include "generators/generators.hh"

#include <QObject>

#include <array>
#include <vector>

typedef void PaStream;

namespace Audio {

class AudioIO;

struct SweepParameters {
    float f0, ff;
    float amplitude;

    float k, log_k;
};

struct AudioData {
    AudioIO *parent;
    std::size_t output_channels_count;
    std::size_t index;
    std::size_t silence_length;
    float amplitude;
    std::vector<float> data_measured;  ///< Measured signal, from the microphone
    std::vector<float> data_reference; ///< Reference signal, going to the speakers
    Generator::Base generator;

    template <typename GeneratorT, typename... Args>
    void init(AudioIO *parent_, std::size_t output_channels_count_, std::size_t silence_length_, float amplitude_,
              Args &&...args) {
        generator.emplace<GeneratorT>(std::forward<Args>(args)...);
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

struct MeasurementData {
    std::vector<float> &reference_signal, &measured_signal;
    Generator::Base generator;
};

class AudioIO : public QObject {
    Q_OBJECT

  public:
    AudioIO();
    ~AudioIO();

    const std::vector<float> &supportedSampleRates();

    // void start(std::unique_ptr<Generator::Base> generator, float volumeDBFS);

    void startSweep(float f0, float ff, std::size_t length, std::size_t sample_rate, float volume_DBFS);

    void startRTA();
    // bool rta_state();

    PaStream *getStream();

    std::vector<float> &getReferenceData();
    std::vector<float> &getMeasuredData();
    MeasurementData getMeasurement();

    void on_audio_finished();

    void handle_rta_data(std::array<float, 24000> data);
    std::array<float, 24000> get_latest_rta_data_();

  signals:
    void audio_finished();
    void on_rta_data();

  private:
    bool init_successful_ = false;
    AudioData data_;
    RTAAudioData rta_data_;
    PaStream *stream_ = nullptr;

    std::vector<float> supported_sample_rates_;
    std::array<float, 24000> latest_rta_data_;
};

} // namespace Audio

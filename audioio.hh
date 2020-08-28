#pragma once

#include <QObject>

#include <vector>

typedef void PaStream;
class AudioIO;

struct SweepParameters {
  float f0, ff;
  float amplitude;

  float k, log_k;
};

struct AudioData {
  AudioIO*           parent;
  std::size_t        output_channels_count;
  std::size_t        index;
  std::size_t        silence_length, length;
  float              sample_rate;
  std::vector<float> data_measured;   // Measured signal, from the microphone
  std::vector<float> data_reference;  // Reference signal, going to the speakers
  SweepParameters    sweep;

  void init(AudioIO*    parent_,
            std::size_t output_channels_count_,
            std::size_t length_,
            std::size_t silence_length_,
            float       sample_rate_,
            float       f0_,
            float       ff_,
            float       amplitude_);
};

struct MeasurementData {
  std::vector<float>&reference_signal, &measured_signal;
  float              sample_rate;
  std::size_t        length;
};

class AudioIO : public QObject {
  Q_OBJECT

 public:
  AudioIO();
  ~AudioIO();

  const std::vector<float>& supportedSampleRates();

  void startSweep(float f0, float ff, std::size_t length, uint32_t sampleRate, float volumeDBFS);

  PaStream* getStream();

  std::vector<float>& getReferenceData();
  std::vector<float>& getMeasuredData();
  MeasurementData     getMeasurement();

  void onAudioFinished();

 signals:
  void audioFinished();

 private:
  bool      mInitSuccessful = false;
  AudioData mData;
  PaStream* mStream = nullptr;

  std::vector<float> mSupportedSampleRates;
};

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

  void init(AudioIO*    parent,
            std::size_t output_channels_count,
            std::size_t length,
            float       sample_rate,
            float       f0,
            float       ff,
            float       amplitude);
};

class AudioIO : public QObject {
  Q_OBJECT

 public:
  AudioIO();
  ~AudioIO();

  std::vector<float>& supportedSampleRates();

  void startSweep(float f0, float ff, std::size_t length, uint32_t sampleRate, float volumeDBFS);

  PaStream*           getStream();
  std::vector<float>& getReferenceData();
  std::vector<float>& getMeasuredData();

  void onAudioFinished();

 signals:
  void audioFinished();

 private:
  bool      mInitSuccessful = false;
  AudioData mData;
  PaStream* mStream;

  std::vector<float> mSupportedSampleRates;
};

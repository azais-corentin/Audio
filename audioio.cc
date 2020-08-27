#include "audioio.hh"

#include <portaudio.h>
#include <spdlog/spdlog.h>
#include <range/v3/action/join.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>

#include <chrono>
#include <memory>
#include <numbers>
#include <vector>

constexpr const float allSampleRates[] = {8000,  11025, 16000, 22050,  44100,
                                          48000, 88200, 96000, 176400, 192000};

void AudioData::init(AudioIO*    parent_,
                     std::size_t output_channels_count_,
                     size_t      length_,
                     float       sample_rate_,
                     float       f0_,
                     float       ff_,
                     float       amplitude_)
{
  parent                = parent_;
  output_channels_count = output_channels_count_;
  index                 = 0;
  silence_length        = 0.1 * sample_rate_;
  length                = length_;
  sample_rate           = sample_rate_;
  data_measured.clear();
  data_measured.reserve(length);
  data_reference.clear();
  data_reference.reserve(length);

  sweep.f0        = f0_;
  sweep.ff        = ff_;
  sweep.amplitude = amplitude_;
  sweep.k         = std::pow(ff_ / f0_, sample_rate / length);
  sweep.log_k     = std::log(sweep.k);
}

static int audioCallback(const void*   input_buffer,
                         void*         output_buffer,
                         unsigned long samples_per_buffer,
                         const PaStreamCallbackTimeInfo* /*time_info*/,
                         PaStreamCallbackFlags /*status_flags*/,
                         void* user_data)
{
  auto data       = static_cast<AudioData*>(user_data);
  auto ro_mic     = static_cast<const float*>(input_buffer);
  auto wo_speaker = static_cast<float*>(output_buffer);

  PaStreamCallbackResult state;
  std::size_t            samples_to_process;
  std::size_t            samples_left = data->silence_length + data->length - data->index;

  if (samples_left < samples_per_buffer) {
    samples_to_process = samples_left;
    state              = paComplete;
  } else {
    samples_to_process = samples_per_buffer;
    state              = paContinue;
  }

  // Read from measured signal and write to AudioData
  if (input_buffer == nullptr) {
    for (std::size_t i = 0; i < samples_to_process; i++)
      data->data_measured.push_back(0);
  } else {
    for (std::size_t i = 0; i < samples_to_process; i++)
      data->data_measured.push_back(*ro_mic++);
  }

  // Write reference signal to speaker & AudioData
  for (std::size_t i = 0; i < samples_to_process; i++) {
    float sample = 0;
    if (data->index + i < data->length) {
      sample = data->sweep.amplitude *
               std::sin(2 * std::numbers::pi_v<float> * data->sweep.f0 *
                        ((std::pow(data->sweep.k, (data->index + i) / data->sample_rate) - 1) /
                         data->sweep.log_k));
    }
    for (std::size_t c = 0; c < data->output_channels_count; c++)
      *wo_speaker++ = sample;
    data->data_reference.push_back(sample);
  }

  if (state == paComplete) {
    // Always output silence at the end
    for (std::size_t c = 0; c < data->output_channels_count; c++)
      *wo_speaker++ = 0;
  }

  data->index += samples_to_process;
  return state;
}

template <double PaDeviceInfo::*m>
static bool lessThan(const PaDeviceInfo* a, const PaDeviceInfo* b)
{
  return a->*m < b->*m;
}

AudioIO::AudioIO()
{
  PaError err;

  auto start = std::chrono::high_resolution_clock::now();

  err = Pa_Initialize();
  if (err != paNoError) {
    spdlog::error("PortAudio error code {}: {}", err, Pa_GetErrorText(err));
    return;
  }

  int framesPerBuffer = 2;

  auto inputDeviceInfo  = Pa_GetDeviceInfo(Pa_GetDefaultInputDevice());
  auto outputDeviceInfo = Pa_GetDeviceInfo(Pa_GetDefaultOutputDevice());

  spdlog::info("input: low {} ms, high {} ms", 1000 * inputDeviceInfo->defaultLowInputLatency,
               1000 * inputDeviceInfo->defaultHighInputLatency);
  spdlog::info("output: low {} ms, high {} ms", 1000 * outputDeviceInfo->defaultLowOutputLatency,
               1000 * outputDeviceInfo->defaultHighOutputLatency);

  auto inputParameters = PaStreamParameters{.device           = Pa_GetDefaultInputDevice(),
                                            .channelCount     = inputDeviceInfo->maxInputChannels,
                                            .sampleFormat     = paFloat32,
                                            .suggestedLatency = 0,
                                            .hostApiSpecificStreamInfo = nullptr};

  auto outputParameters = PaStreamParameters{.device       = Pa_GetDefaultOutputDevice(),
                                             .channelCount = outputDeviceInfo->maxOutputChannels,
                                             .sampleFormat = paFloat32,
                                             .suggestedLatency          = 0,
                                             .hostApiSpecificStreamInfo = nullptr};

  auto step_init = std::chrono::high_resolution_clock::now();

  for (auto sampleRate : allSampleRates) {
    PaStream* stream;
    err = Pa_OpenStream(
        &stream, &inputParameters, &outputParameters, sampleRate, framesPerBuffer, paClipOff,
        [](const void*, void*, unsigned long, const PaStreamCallbackTimeInfo*,
           PaStreamCallbackFlags, void*) -> int { return paContinue; },
        nullptr);
    if (err == paInvalidSampleRate) {
      continue;
    }
    if (err != paNoError && err != paInvalidSampleRate) {
      spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
      return;
    }
    if (err == paNoError) {
      mSupportedSampleRates.push_back(sampleRate);
    }
    Pa_CloseStream(stream);
  }

  auto t1 =
      mSupportedSampleRates |
      ranges::views::transform([](float sampleRate) { return fmt::format("{:.0f}", sampleRate); }) |
      ranges::to_vector;
  auto sSR = t1 | ranges::views::join(", ") | ranges::to<std::string>;

  spdlog::info("Supported sample rates: {}", sSR);

  auto step_ssr = std::chrono::high_resolution_clock::now();

  PaStream* stream;

  err = Pa_OpenStream(
      &stream, &inputParameters, &outputParameters, mSupportedSampleRates.back(), framesPerBuffer,
      paClipOff,
      [](const void*, void*, unsigned long, const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags,
         void*) -> int { return paContinue; },
      nullptr);
  if (err != paNoError) {
    spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
    return;
  }

  const PaStreamInfo* streamInfo;
  streamInfo = Pa_GetStreamInfo(stream);

  spdlog::info("Using input device #{} ({}, {})", inputParameters.device, inputDeviceInfo->name,
               Pa_GetHostApiInfo(inputDeviceInfo->hostApi)->name);
  spdlog::info("Using output device #{} ({}, {})", outputParameters.device, outputDeviceInfo->name,
               Pa_GetHostApiInfo(outputDeviceInfo->hostApi)->name);

  err = Pa_CloseStream(stream);
  if (err != paNoError) {
    spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
    return;
  }

  auto step_latency = std::chrono::high_resolution_clock::now();

  mInitSuccessful = true;

  spdlog::info(
      "Initialized in {:.2f} ms: init {:.2f} ms + sample rates {:.2f} ms + latency {:.2f} ms",
      (step_latency - start).count() / 1000000., (step_init - start).count() / 1000000.,
      (step_ssr - step_init).count() / 1000000., (step_latency - step_ssr).count() / 1000000.);
}

AudioIO::~AudioIO()
{
  PaError err = Pa_Terminate();
  if (err != paNoError)
    spdlog::error("PortAudio error code {}: {}", err, Pa_GetErrorText(err));
}

void AudioIO::startSweep(float       f0,
                         float       ff,
                         std::size_t length,
                         uint32_t    sampleRate,
                         float       volumeDBFS)
{
  // Validate sample rate
  if (std::find(mSupportedSampleRates.begin(), mSupportedSampleRates.end(), sampleRate) ==
      std::end(mSupportedSampleRates)) {
    spdlog::error("Unsupported sample rate: {}", sampleRate);
    return;
  }
  // Validate volume
  if (volumeDBFS > 0) {
    spdlog::error("Volume {} dBFS cannot be above 0", volumeDBFS);
    return;
  }

  // Setup input & output parameters
  auto inputDeviceInfo  = Pa_GetDeviceInfo(Pa_GetDefaultInputDevice());
  auto outputDeviceInfo = Pa_GetDeviceInfo(Pa_GetDefaultOutputDevice());

  auto inputParameters =
      PaStreamParameters{.device                    = Pa_GetDefaultInputDevice(),
                         .channelCount              = 1,
                         .sampleFormat              = paFloat32,
                         .suggestedLatency          = inputDeviceInfo->defaultLowInputLatency,
                         .hostApiSpecificStreamInfo = nullptr};

  auto outputParameters =
      PaStreamParameters{.device                    = Pa_GetDefaultOutputDevice(),
                         .channelCount              = 2,
                         .sampleFormat              = paFloat32,
                         .suggestedLatency          = outputDeviceInfo->defaultLowOutputLatency,
                         .hostApiSpecificStreamInfo = nullptr};

  // Measure latency
  PaTime inputLatency, outputLatency;
  {
    PaStream* stream;

    PaError err = Pa_OpenStream(
        &stream, &inputParameters, &outputParameters, sampleRate, 2, paClipOff,
        [](const void*, void*, unsigned long, const PaStreamCallbackTimeInfo*,
           PaStreamCallbackFlags, void*) -> int { return paContinue; },
        nullptr);
    if (err != paNoError) {
      spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
      return;
    }

    PaStreamInfo streamInfo = *Pa_GetStreamInfo(stream);
    inputLatency            = streamInfo.inputLatency;
    outputLatency           = streamInfo.outputLatency;
    err                     = Pa_CloseStream(stream);
    if (err != paNoError) {
      spdlog::error("Pa_CloseStream error #{}: {} ", err, Pa_GetErrorText(err));
      return;
    }
  }
  spdlog::info("Measured latency: input {} ms / ouput {} ms", inputLatency * 1000,
               outputLatency * 1000);

  // Configure audio data
  mData.init(this, 2, length, sampleRate, f0, ff, std::pow(10, volumeDBFS / 20.));

  // Configure stream
  PaError err;

  err = Pa_OpenStream(&mStream, &inputParameters, &outputParameters, sampleRate,
                      paFramesPerBufferUnspecified, paClipOff, audioCallback, &mData);
  if (err != paNoError) {
    spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
    return;
  }

  err = Pa_StartStream(mStream);
  if (err != paNoError) {
    spdlog::error("Pa_StartStream error #{}: {} ", err, Pa_GetErrorText(err));
    return;
  }

  spdlog::info("Hardware sample rate: {}", Pa_GetStreamInfo(mStream)->sampleRate);

  Pa_SetStreamFinishedCallback(mStream, [](void* userData) {
    spdlog::info("Pa_SetStreamFinishedCallback");
    auto data = static_cast<AudioData*>(userData);
    data->parent->onAudioFinished();
    PaError err = Pa_CloseStream(data->parent->getStream());
    if (err != paNoError) {
      spdlog::error("Pa_CloseStream error #{}: {} ", err, Pa_GetErrorText(err));
      return;
    }
  });
}

PaStream* AudioIO::getStream()
{
  return mStream;
}

std::vector<float>& AudioIO::getReferenceData()
{
  return mData.data_reference;
}

std::vector<float>& AudioIO::getMeasuredData()
{
  return mData.data_measured;
}

void AudioIO::onAudioFinished()
{
  emit audioFinished();
}

#include "audioio.hh"

#include <portaudio.h>
#include <spdlog/spdlog.h>
#include <range/v3/action/join.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>

#include <chrono>
#include <concepts>
#include <memory>
#include <numbers>
#include <vector>

namespace Audio {

constexpr const float allSampleRates[] = {8000,  11025, 16000, 22050,  44100,
                                          48000, 88200, 96000, 176400, 192000};

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

  return std::visit(
      [&](auto& generator) {
        PaStreamCallbackResult state;
        std::size_t            samples_to_process;
        std::size_t            samples_left =
            generator.length() + data->silence_length - data->index;

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
          std::size_t index  = data->index + i;
          float       sample = data->amplitude * generator.sample(index);

          data->data_reference.push_back(sample);
          for (std::size_t c = 0; c < data->output_channels_count; c++)
            *wo_speaker++ = sample;
        }

        if (state == paComplete) {
          // Always output silence at the end
          /*for (std::size_t c = 0; c < data->output_channels_count; c++)
           *wo_speaker++ = 0;*/
        }

        data->index += samples_to_process;
        return state;
      },
      data->generator);
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

  spdlog::info("input: low {} ms, high {} ms",
               1000 * inputDeviceInfo->defaultLowInputLatency,
               1000 * inputDeviceInfo->defaultHighInputLatency);
  spdlog::info("output: low {} ms, high {} ms",
               1000 * outputDeviceInfo->defaultLowOutputLatency,
               1000 * outputDeviceInfo->defaultHighOutputLatency);

  auto inputParameters =
      PaStreamParameters{.device                    = Pa_GetDefaultInputDevice(),
                         .channelCount              = inputDeviceInfo->maxInputChannels,
                         .sampleFormat              = paFloat32,
                         .suggestedLatency          = 0,
                         .hostApiSpecificStreamInfo = nullptr};

  auto outputParameters =
      PaStreamParameters{.device                    = Pa_GetDefaultOutputDevice(),
                         .channelCount              = outputDeviceInfo->maxOutputChannels,
                         .sampleFormat              = paFloat32,
                         .suggestedLatency          = 0,
                         .hostApiSpecificStreamInfo = nullptr};

  auto step_init = std::chrono::high_resolution_clock::now();

  for (auto sampleRate : allSampleRates) {
    PaStream* stream;
    err = Pa_OpenStream(
        &stream, &inputParameters, &outputParameters, sampleRate, framesPerBuffer,
        paClipOff,
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

  auto t1 = mSupportedSampleRates | ranges::views::transform([](float sampleRate) {
              return fmt::format("{:.0f}", sampleRate);
            }) |
            ranges::to_vector;
  auto sSR = t1 | ranges::views::join(", ") | ranges::to<std::string>;

  spdlog::info("Supported sample rates: {}", sSR);

  auto step_ssr = std::chrono::high_resolution_clock::now();

  PaStream* stream;

  err = Pa_OpenStream(
      &stream, &inputParameters, &outputParameters, mSupportedSampleRates.back(),
      framesPerBuffer, paClipOff,
      [](const void*, void*, unsigned long, const PaStreamCallbackTimeInfo*,
         PaStreamCallbackFlags, void*) -> int { return paContinue; },
      nullptr);
  if (err != paNoError) {
    spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
    return;
  }

  const PaStreamInfo* streamInfo;
  streamInfo = Pa_GetStreamInfo(stream);

  spdlog::info("Using input device #{} ({}, {})", inputParameters.device,
               inputDeviceInfo->name, Pa_GetHostApiInfo(inputDeviceInfo->hostApi)->name);
  spdlog::info("Using output device #{} ({}, {})", outputParameters.device,
               outputDeviceInfo->name,
               Pa_GetHostApiInfo(outputDeviceInfo->hostApi)->name);

  err = Pa_CloseStream(stream);
  if (err != paNoError) {
    spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
    return;
  }

  auto step_latency = std::chrono::high_resolution_clock::now();

  mInitSuccessful = true;

  spdlog::info(
      "Initialized in {:.2f} ms: init {:.2f} ms + sample rates {:.2f} ms + latency "
      "{:.2f} ms",
      (step_latency - start).count() / 1000000., (step_init - start).count() / 1000000.,
      (step_ssr - step_init).count() / 1000000.,
      (step_latency - step_ssr).count() / 1000000.);
}

AudioIO::~AudioIO()
{
  PaError err = Pa_Terminate();
  if (err != paNoError)
    spdlog::error("PortAudio error code {}: {}", err, Pa_GetErrorText(err));
}

const std::vector<float>& AudioIO::supportedSampleRates()
{
  return mSupportedSampleRates;
}

void AudioIO::startSweep(float       f0,
                         float       ff,
                         std::size_t length,
                         std::size_t sampleRate,
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
  // Check we're not already sweeping
  if (Pa_IsStreamActive(mStream) == 1) {
    spdlog::error("Previous sweep is not finished");
    return;
  }

  // Setup input & output parameters
  auto inputDeviceInfo  = Pa_GetDeviceInfo(Pa_GetDefaultInputDevice());
  auto outputDeviceInfo = Pa_GetDeviceInfo(Pa_GetDefaultOutputDevice());

  auto inputParameters =
      PaStreamParameters{.device           = Pa_GetDefaultInputDevice(),
                         .channelCount     = 1,
                         .sampleFormat     = paFloat32,
                         .suggestedLatency = inputDeviceInfo->defaultLowInputLatency,
                         .hostApiSpecificStreamInfo = nullptr};

  auto outputParameters =
      PaStreamParameters{.device           = Pa_GetDefaultOutputDevice(),
                         .channelCount     = 2,
                         .sampleFormat     = paFloat32,
                         .suggestedLatency = outputDeviceInfo->defaultLowOutputLatency,
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

  // Remove 50 ms to the length to allow reflections to get into the measured signals
  length -= 0.05 * sampleRate;

  // Configure audio data
  // Silence = 100ms + input & output latency
  mData.init<Generator::SynchronizedSweptSine>(
      this, 2, (0.1 + inputLatency + outputLatency) * sampleRate,
      std::pow(10, volumeDBFS / 20.), f0, ff, length, sampleRate);

  // Configure stream
  PaError err;

  err = Pa_OpenStream(&mStream, &inputParameters, &outputParameters, sampleRate,
                      paFramesPerBufferUnspecified, paClipOff, audioCallback, &mData);
  if (err != paNoError) {
    spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
    return;
  }

  err = Pa_SetStreamFinishedCallback(mStream, [](void* userData) {
    static_cast<AudioData*>(userData)->parent->onAudioFinished();
  });
  if (err != paNoError) {
    spdlog::error("Pa_SetStreamFinishedCallback error #{}: {} ", err,
                  Pa_GetErrorText(err));
    return;
  }

  err = Pa_StartStream(mStream);
  if (err != paNoError) {
    spdlog::error("Pa_StartStream error #{}: {} ", err, Pa_GetErrorText(err));
    return;
  }

  spdlog::info("Hardware sample rate: {}", Pa_GetStreamInfo(mStream)->sampleRate);
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

MeasurementData AudioIO::getMeasurement()
{
  return MeasurementData{
      .reference_signal = mData.data_reference,
      .measured_signal  = mData.data_measured,
      .generator        = mData.generator,
  };
}

void AudioIO::onAudioFinished()
{
  spdlog::info(mData.data_measured.size());
  spdlog::info(mData.data_reference.size());
  spdlog::info(mData.data_measured.capacity());
  spdlog::info(mData.data_reference.capacity());

  emit audioFinished();
}

}  // namespace Audio

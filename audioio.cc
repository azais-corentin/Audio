#include "audioio.hh"

#include <portaudio.h>
#include <spdlog/spdlog.h>

#include <vector>

AudioIO::AudioIO()
{
  PaError err;

  err = Pa_Initialize();
  if (err != paNoError) {
    spdlog::error("PortAudio error code {}: {}", err, Pa_GetErrorText(err));
    return;
  }

  int numDevices;
  numDevices = Pa_GetDeviceCount();
  if (numDevices < 0) {
    err = numDevices;
    spdlog::error("PortAudio error on Pa_CountDevices code {}: {}", err,
                  Pa_GetErrorText(err));
    return;
  }

  std::vector<const PaDeviceInfo*> inputDevicesInfo, outputDevicesInfo;

  for (int i = 0; i < numDevices; i++) {
    auto deviceInfo = Pa_GetDeviceInfo(i);
    if (deviceInfo->maxInputChannels > 0) {
      inputDevicesInfo.push_back(deviceInfo);
    }
    if (deviceInfo->maxOutputChannels > 0) {
      outputDevicesInfo.push_back(deviceInfo);
    }

    spdlog::info("device {}: {}", i, deviceInfo->name);
    spdlog::info("maxInputChannels: {}", deviceInfo->maxInputChannels);
    spdlog::info("defaultSampleRate: {}", deviceInfo->defaultSampleRate);
    spdlog::info("maxOutputChannels: {}", deviceInfo->maxOutputChannels);
    spdlog::info("defaultLowInputLatency: {}", deviceInfo->defaultLowInputLatency);
    spdlog::info("defaultHighInputLatency: {}", deviceInfo->defaultHighInputLatency);
    spdlog::info("defaultLowOutputLatency: {}", deviceInfo->defaultLowOutputLatency);
    spdlog::info("defaultHighOutputLatency: {}", deviceInfo->defaultHighOutputLatency);
    spdlog::info("");

    /*auto f = [](auto instance, auto m) { return instance.*m; };

    auto test = f(*deviceInfo, &PaDeviceInfo::defaultLowOutputLatency);
    spdlog::info("test: {}", test);*/
  }

  auto bestInputDevice = *std::min_element(
      inputDevicesInfo.begin(), inputDevicesInfo.end(),
      [](const PaDeviceInfo* devAinfo, const PaDeviceInfo* devBinfo) {
        return devAinfo->defaultLowInputLatency < devBinfo->defaultLowInputLatency;
      });

  auto bestOutputDevice = *std::min_element(
      outputDevicesInfo.begin(), outputDevicesInfo.end(),
      [](const PaDeviceInfo* devAinfo, const PaDeviceInfo* devBinfo) {
        return devAinfo->defaultLowOutputLatency < devBinfo->defaultLowOutputLatency;
      });

  spdlog::info("Selected input device: {} with latency {} ms", bestInputDevice->name,
               bestInputDevice->defaultLowInputLatency * 1000.);
  spdlog::info("Selected output device: {} with latency {} ms", bestOutputDevice->name,
               bestOutputDevice->defaultLowOutputLatency * 1000.);

  mInitSuccessful = true;
}

AudioIO::~AudioIO()
{
  PaError err = Pa_Terminate();
  if (err != paNoError)
    spdlog::error("PortAudio error code {}: {}", err, Pa_GetErrorText(err));
}

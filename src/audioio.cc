#include "audioio.hh"

#include <RtAudio.h>
#include <fmt/format.h>
#include <magic_enum.hpp>
#include <range/v3/action/join.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <memory>
#include <vector>

namespace Audio {

int audioCallback(
    void *outputBuffer, void *inputBuffer, unsigned int nFrames, double /*streamTime*/,
    RtAudioStreamStatus status, void *userData
)
{
    if(status) {
        if(status == RTAUDIO_INPUT_OVERFLOW) {
            spdlog::error("Stream input overflow detected: input data discarded");
        }
        if(status == RTAUDIO_OUTPUT_UNDERFLOW) {
            spdlog::error("Stream output underflow detected: output data missing");
        }
    }

    auto &data = *static_cast<SweepData *>(userData);

    if(not data.generator) {
        spdlog::error("Invalid generator in audioCallback");
        return 2;
    }

    const auto outputChannelCount = data.outputChannelCount;
    const auto inputChannelCount  = data.inputChannelCount;

    std::span<float> output{static_cast<float *>(outputBuffer), nFrames * outputChannelCount};
    std::span<const float> input{
        static_cast<float const *>(inputBuffer), nFrames * inputChannelCount
    };

    // We have (nFrames * channelCount) floats available in the buffers for input and output

    // Write reference
    for(uint32_t iFrame = 0; iFrame < nFrames; iFrame++) {
        auto iSample = iFrame * outputChannelCount;
        auto sample  = data.generator->sample(data.index++);

        for(uint32_t k = 0; k < outputChannelCount; k++) { output[iSample + k] = sample; }
        data.referenceData.push_back(sample);
    }

    // Read measured from first channel only
    for(uint32_t iFrame = 0; iFrame < nFrames; iFrame++) {
        auto iSample = iFrame * inputChannelCount;
        data.measuredData.push_back(input[iSample]);
    }

    // 0 continues the stream, 2 aborts the stream
    if(data.generator->finished(data.index)) {
        data.parent->stopSweep();
        return 2;
    }
    return 0;
}

AudioIO::AudioIO() : audio_{std::make_unique<RtAudio>()}
{
    auto timeDeviceInfo = std::chrono::high_resolution_clock::now();

    if(audio_->getDeviceIds().size() < 1) {
        spdlog::error("No audio device available");
        std::exit(-1);
    }

    auto inputDeviceInfo  = audio_->getDeviceInfo(audio_->getDefaultInputDevice());
    auto outputDeviceInfo = audio_->getDeviceInfo(audio_->getDefaultOutputDevice());

    spdlog::info(
        "Using {} ({})", audio_->getApiDisplayName(audio_->getCurrentApi()),
        audio_->getApiName(audio_->getCurrentApi())
    );
    spdlog::info("Input details:");
    spdlog::info("\tDevice #{}: {}", inputDeviceInfo.ID, inputDeviceInfo.name);
    spdlog::info("\tSupported sample rates: {}", fmt::join(inputDeviceInfo.sampleRates, ", "));
    spdlog::info("\tPreferred sample rate: {}", inputDeviceInfo.preferredSampleRate);
    spdlog::info("\tSupported input formats:");
    if(inputDeviceInfo.nativeFormats & RTAUDIO_SINT8) { spdlog::info("\t\t8-bit signed integer"); }
    if(inputDeviceInfo.nativeFormats & RTAUDIO_SINT16) {
        spdlog::info("\t\t16-bit signed integer");
    }
    if(inputDeviceInfo.nativeFormats & RTAUDIO_SINT24) {
        spdlog::info("\t\t24-bit signed integer");
    }
    if(inputDeviceInfo.nativeFormats & RTAUDIO_SINT32) {
        spdlog::info("\t\t32-bit signed integer");
    }
    if(inputDeviceInfo.nativeFormats & RTAUDIO_FLOAT32) {
        spdlog::info("\t\tsingle precision floating point");
    } else {
        spdlog::error("Single precision floating point format not supported");
        std::exit(-1);
    }
    if(inputDeviceInfo.nativeFormats & RTAUDIO_FLOAT64) {
        spdlog::info("\t\tdouble precision floating point");
    }

    spdlog::info("Output details:");
    spdlog::info("\tDevice #{}: {}", outputDeviceInfo.ID, outputDeviceInfo.name);
    spdlog::info("\tSupported sample rates: {}", fmt::join(outputDeviceInfo.sampleRates, ", "));
    spdlog::info("\tPreferred sample rate: {}", outputDeviceInfo.preferredSampleRate);
    spdlog::info("\tSupported output formats:");
    if(outputDeviceInfo.nativeFormats & RTAUDIO_SINT8) { spdlog::info("\t\t8-bit signed integer"); }
    if(outputDeviceInfo.nativeFormats & RTAUDIO_SINT16) {
        spdlog::info("\t\t16-bit signed integer");
    }
    if(outputDeviceInfo.nativeFormats & RTAUDIO_SINT24) {
        spdlog::info("\t\t24-bit signed integer");
    }
    if(outputDeviceInfo.nativeFormats & RTAUDIO_SINT32) {
        spdlog::info("\t\t32-bit signed integer");
    }
    if(outputDeviceInfo.nativeFormats & RTAUDIO_FLOAT32) {
        spdlog::info("\t\tsingle precision floating point");
    } else {
        spdlog::error("Single precision floating point format not supported");
        std::exit(-1);
    }
    if(outputDeviceInfo.nativeFormats & RTAUDIO_FLOAT64) {
        spdlog::info("\t\tdouble precision floating point");
    }

    auto timeLatency = std::chrono::high_resolution_clock::now();

    std::set_intersection(
        inputDeviceInfo.sampleRates.begin(), inputDeviceInfo.sampleRates.end(),
        inputDeviceInfo.sampleRates.begin(), inputDeviceInfo.sampleRates.end(),
        std::back_inserter(supportedSampleRates_)
    );

    const auto sampleRate = supportedSampleRates_.back(); // Select highest available sample rate

    RtAudio::StreamParameters parameters;
    parameters.deviceId       = audio_->getDefaultOutputDevice();
    parameters.nChannels      = 2;
    unsigned int bufferFrames = 256; // 256 sample frames

    RtAudio::StreamOptions options;
    options.flags = RTAUDIO_NONINTERLEAVED | RTAUDIO_MINIMIZE_LATENCY;

    spdlog::info("Opening 2-channel stream at {} Hz", sampleRate);

    if(audio_->openStream(
           &parameters, nullptr, RTAUDIO_FLOAT32, sampleRate, &bufferFrames,
           [](void *, void *, unsigned int, double, RtAudioStreamStatus, void *) -> int {
        return 0;
    },
           NULL, &options
       ) != RTAUDIO_NO_ERROR) {
        spdlog::error("{}", audio_->getErrorText());
        std::exit(-1);
    }

    spdlog::info(
        "Stream latency: {} frames = {} ms", audio_->getStreamLatency(),
        (1000.F * audio_->getStreamLatency()) / audio_->getStreamSampleRate()
    );

    if(audio_->isStreamOpen()) { audio_->closeStream(); }

    // err = Pa_OpenStream(
    //     &stream, &inputParameters, &outputParameters, supported_sample_rates_.back(),
    //     static_cast<uint32_t>(framesPerBuffer), paClipOff,
    //     [](const void *, void *, unsigned long, const PaStreamCallbackTimeInfo *,
    //     PaStreamCallbackFlags,
    //        void *) -> int { return paContinue; },
    //     nullptr);
    // if (err != paNoError) {
    //     spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
    //     return;
    // }

    // err = Pa_CloseStream(stream);
    // if (err != paNoError) {
    //     spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
    //     return;
    // }

    auto timeFinal = std::chrono::high_resolution_clock::now();

    spdlog::info(
        "Initialized in {:.2f} ms (audio: {:.2f} ms, info: {:.2f} ms, latency: {:.2} ms)",
        static_cast<double>((timeFinal - timeStart_).count()) / 1e6,
        static_cast<double>((timeDeviceInfo - timeStart_).count()) / 1e6,
        static_cast<double>((timeLatency - timeDeviceInfo).count()) / 1e6,
        static_cast<double>((timeFinal - timeLatency).count()) / 1e6
    );
}

AudioIO::~AudioIO() = default;

std::span<const uint32_t> AudioIO::supportedSampleRates() { return supportedSampleRates_; }

// void AudioIO::start(std::unique_ptr<Generator::Base> generator, float volumeDBFS) {}

void AudioIO::startSweep(
    float frequencyBegin, float frequencyEnd, uint32_t length, uint32_t silenceLength,
    uint32_t sampleRate, float volumeDBFS
)
{
    // Validate frequencies
    if(frequencyBegin >= frequencyEnd) {
        spdlog::error("frequencyBegin must be less than frequencyEnd");
        return;
    }
    // Validate lengths
    if(length == 0) { spdlog::error("Length should not be zero"); }
    if(silenceLength == 0) { spdlog::error("Silence length should not be zero"); }
    // Validate sample rate
    if(std::find(supportedSampleRates_.begin(), supportedSampleRates_.end(), sampleRate) ==
       std::end(supportedSampleRates_)) {
        spdlog::error("Unsupported sample rate: {}", sampleRate);
        return;
    }
    // Validate volume
    if(volumeDBFS > 0) {
        spdlog::error("Volume {} dBFS cannot be above 0", volumeDBFS);
        return;
    }
    // Check we're not already sweeping
    if(audio_->isStreamOpen() or audio_->isStreamRunning()) {
        spdlog::error("Previous sweep is not finished");
        return;
    }

    // Configure sweep data
    const float amplitude = std::pow(10.F, volumeDBFS / 20.F);
    sweepData_            = SweepData{
        this,
        std::make_unique<Generators::SynchronizedSweptSine>(
            frequencyBegin, frequencyEnd, length, silenceLength, sampleRate, amplitude
        ),
        2, 1
    };

    // Setup audio stream
    uint32_t bufferFrames = 512; // ~10ms buffer at 48 kHz
    RtAudio::StreamParameters inputParameters, outputParameterss;
    inputParameters.deviceId    = audio_->getDefaultInputDevice();
    inputParameters.nChannels   = 1;
    outputParameterss.deviceId  = audio_->getDefaultOutputDevice();
    outputParameterss.nChannels = 2;

    if(audio_->openStream(
           &outputParameterss, &inputParameters, RTAUDIO_FLOAT32, sampleRate, &bufferFrames,
           &audioCallback, &sweepData_
       ) != RTAUDIO_NO_ERROR) {
        spdlog::error("Failed to create sweep: {}", audio_->getErrorText());
        if(audio_->isStreamOpen()) { audio_->closeStream(); }
        return;
    }

    if(audio_->startStream() != RTAUDIO_NO_ERROR) {
        spdlog::error("Failed to start sweep: {}", audio_->getErrorText());
        if(audio_->isStreamOpen()) { audio_->closeStream(); }
        return;
    }
}

// void AudioIO::startRTA()
// {
//     std::size_t sample_rate = 48'000;

//     // Validate sample rate
//     if(std::find(supportedSampleRates_.begin(), supportedSampleRates_.end(), sample_rate) ==
//        std::end(supportedSampleRates_)) {
//         spdlog::error("Unsupported sample rate: {}", sample_rate);
//         return;
//     }
//     // Check we're not already sweeping
//     /*
//     if (Pa_IsStreamActive(stream_) == 1) {
//         spdlog::error("Another stream is active, closing!");
//         Pa_CloseStream(stream_);
//         return;
//     }

//     // Setup input parameters
//     auto inputDeviceInfo = Pa_GetDeviceInfo(Pa_GetDefaultInputDevice());

//     auto inputParameters = PaStreamParameters{.device                    =
//     Pa_GetDefaultInputDevice(), .channelCount              = 1, .sampleFormat              =
//     paFloat32, .suggestedLatency          = inputDeviceInfo->defaultLowInputLatency,
//                                               .hostApiSpecificStreamInfo = nullptr};

//     rta_data_.init(this);

//     // Configure stream
//     PaError err;

//     err = Pa_OpenStream(&stream_, &inputParameters, nullptr, static_cast<double>(sample_rate),
//                         paFramesPerBufferUnspecified, paClipOff, audio_rta_callback, &rta_data_);
//     if (err != paNoError) {
//         spdlog::error("Pa_OpenStream error #{}: {} ", err, Pa_GetErrorText(err));
//         return;
//     }

//     err = Pa_StartStream(stream_);
//     if (err != paNoError) {
//         spdlog::error("Pa_StartStream error #{}: {} ", err, Pa_GetErrorText(err));
//         return;
//     }

//     spdlog::info("Hardware sample rate: {}", Pa_GetStreamInfo(stream_)->sampleRate);
//     */
// }

const SweepData &AudioIO::getSweepData() { return sweepData_; }

void AudioIO::stopSweep() // Called by audioCallback when done
{
    spdlog::info("Reference size: {}", sweepData_.referenceData.size());
    spdlog::info("Measured size: {}", sweepData_.measuredData.size());

    if(not audio_->isStreamOpen()) { return; }

    if(audio_->isStreamRunning()) { audio_->abortStream(); }
    if(audio_->isStreamOpen()) { audio_->closeStream(); }

    emit sweepFinished();
}

// void AudioIO::handle_rta_data(std::array<float, 24'000> data)
// {
//     latestRTAData_ = data;
//     emit on_rta_data();
// }

// std::array<float, 24'000> AudioIO::get_latest_rta_data_() { return latestRTAData_; }

} // namespace Audio

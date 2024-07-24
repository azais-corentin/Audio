#include "mainwindow.hh"

#include "./ui_mainwindow.h"
#include "fft.hh"
#include "timedelay.hh"

#include <QDebug>
#include <QtAwesome.h>
#include <qspinbox.h>
#include <range/v3/action/drop.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <execution>
#include <memory>

namespace Audio
{

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui_{new Ui::MainWindow}, awesome_{std::make_unique<fa::QtAwesome>()}
{
    ui_->setupUi(this);
    setupUi();

    awesome_->initFontAwesome();

    // connect(&audio_, &AudioIO::audio_finished, this, &MainWindow::handleFinished);
    // connect(&audio_, &AudioIO::on_rta_data, this, &MainWindow::handleRtaData);
}

MainWindow::~MainWindow() { delete ui_; }

void MainWindow::setupUi()
{
    // {
    //     quickPlot_.addGraph();
    //     quickPlot_.addGraph();
    //     quickPlot_.graph(0)->setPen({Qt::blue});
    //     quickPlot_.graph(1)->setPen({Qt::red});
    //     quickPlot_.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    //     quickPlot_.xAxis->grid()->setSubGridVisible(true);
    //     quickPlot_.axisRect()->setupFullAxesBox();
    //     connect(
    //         quickPlot_.xAxis, qOverload<const QCPRange &>(&QCPAxis::rangeChanged),
    //         quickPlot_.xAxis2, qOverload<const QCPRange &>(&QCPAxis::setRange)
    //     );
    //     connect(
    //         quickPlot_.yAxis, qOverload<const QCPRange &>(&QCPAxis::rangeChanged),
    //         quickPlot_.yAxis2, qOverload<const QCPRange &>(&QCPAxis::setRange)
    //     );
    // }

    setupPlotForLogLine(*ui_->plotResponse);

    // Measurement parameters
    {
        // Get supported sample rates
        auto supportedSampleRates = audio_.supportedSampleRates();
        for(auto sampleRate : supportedSampleRates) {
            ui_->eSampleRate->addItem(QString::number(sampleRate), sampleRate);
        }

        // Select first sample rate above 45 kHz or highest otherwise
        const auto bestSampleRate = std::find_if(
            supportedSampleRates.begin(), supportedSampleRates.end(),
            [](uint32_t sampleRate) { return sampleRate > 45'000; }
        );
        if(bestSampleRate != supportedSampleRates.end()) {
            ui_->eSampleRate->setCurrentIndex(
                std::distance(supportedSampleRates.begin(), bestSampleRate)
            );
        } else {
            ui_->eSampleRate->setCurrentIndex(ui_->eSampleRate->count() - 1);
        }

        // Length
        ui_->eLength->addItem("128k", (1U << 17U));
        ui_->eLength->addItem("256k", (1U << 18U));
        ui_->eLength->addItem("512k", (1U << 19U));
        ui_->eLength->addItem("1M", (1U << 20U));
        ui_->eLength->setCurrentIndex(1); // Select 256k by default
    }

    // Setup colormap
    {
        auto &plot = *ui_->plotOutputSpectrogram;

        plot.setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

        // Title
        QFont titleFont;
        titleFont.setBold(true);
        titleFont.setPointSize(12);
        plot.plotLayout()->insertRow(0);
        plot.plotLayout()->addElement(
            0, 0, new QCPTextElement(&plot, "Input spectrogram", titleFont)
        );

        // Axes
        plot.axisRect()->setupFullAxesBox(true);
        plot.xAxis->setLabel("Time [s]");
        plot.yAxis->setLabel("Frequency [Hz]");
        plot.yAxis->setScaleType(QCPAxis::stLogarithmic);
        plot.yAxis->setTicker(QSharedPointer<QCPAxisTickerLog>{new QCPAxisTickerLog});

        // Color map
        colorMap_ = std::make_unique<QCPColorMap>(plot.xAxis, plot.yAxis);
        colorMap_->data()->setSize(200, 200);
        colorMap_->data()->setRange(QCPRange(0, 8), QCPRange(20, 20'000));

        double x, y, z;
        for(int xIndex = 0; xIndex < 200; ++xIndex) {
            for(int yIndex = 0; yIndex < 200; ++yIndex) {
                colorMap_->data()->cellToCoord(xIndex, yIndex, &x, &y);
                double r = 3 * qSqrt(x * x + y * y) + 1e-2;
                z        = 2 * x * (qCos(r + 2) / r - qSin(r + 2) / r);
                colorMap_->data()->setCell(xIndex, yIndex, z);
            }
        }

        // Add color scale
        colorScale_ = std::make_unique<QCPColorScale>(&plot);
        plot.plotLayout()->addElement(1, 1, colorScale_.get());
        colorScale_->setType(QCPAxis::atRight);
        colorMap_->setColorScale(colorScale_.get());
        colorScale_->axis()->setLabel("Magnetic Field Strength");

        colorMap_->setGradient(QCPColorGradient::gpPolar);
        colorMap_->rescaleDataRange();

        // Align color scale with axis rect
        marginGroup_ = std::make_unique<QCPMarginGroup>(&plot);
        plot.axisRect()->setMarginGroup(QCP::msBottom | QCP::msTop, marginGroup_.get());
        colorScale_->setMarginGroup(QCP::msBottom | QCP::msTop, marginGroup_.get());

        plot.rescaleAxes();
    }

    // Connections
    connect(ui_->eBeginFrequency, &QSpinBox::valueChanged, this, &MainWindow::updateStartFrequency);
    connect(ui_->eEndFrequency, &QSpinBox::valueChanged, this, &MainWindow::updateEndFrequency);
    connect(
        ui_->eSampleRate, qOverload<int>(&QComboBox::currentIndexChanged), this,
        &MainWindow::updateMeasurementDuration
    );
    connect(
        ui_->eLength, qOverload<int>(&QComboBox::currentIndexChanged), this,
        &MainWindow::updateMeasurementDuration
    );
    connect(ui_->bMeasure, &QPushButton::clicked, this, &MainWindow::startMeasurement);

    // Update ui
    updateMeasurementDuration();
    ui_->plotResponse->xAxis->setRange(ui_->eBeginFrequency->value(), ui_->eEndFrequency->value());
}

// void MainWindow::quickPlot(std::string name, const std::vector<float> &signal, float scaling)
// {
//     std::vector<float> signal_t(signal.size());
//     std::generate(signal_t.begin(), signal_t.end(), [&, n = 0]() mutable { return n++ * scaling;
//     });

//     auto *graph      = quickPlot_.addGraph();
//     const auto index = quickPlot_.graphCount() - 1;
//     graph->setPen(QColor::fromHslF(std::fmod(index * 0.618033988749895f, 1.0f), 1.0, 0.5));
//     graph->setName(QString::fromStdString(name));

//     graph->setData(
//         QVector<double>(signal_t.begin(), signal_t.end()),
//         QVector<double>(signal.begin(), signal.end())
//     );
//     quickPlot_.replot();
//     quickPlot_.rescaleAxes();
//     quickPlot_.show();
//     quickPlot_.legend->setVisible(true);
// }

void MainWindow::handleFinished()
{
    // quickPlot_.clearGraphs();
    using namespace ranges;

    auto const &sweepData = audio_.getSweepData();

    // Get measurement parameters
    // const float frequencyBegin = ui_->eBeginFrequency->value();
    // const float frequencyEnd   = ui_->eEndFrequency->value();
    // const std::size_t full_length = ui_->eLength->currentData().toUInt();
    // const std::size_t length      = std::visit([](const auto &g) { return g.length();
    // }, measurement.generator);
    const auto sample_rate = sweepData.generator->sampleRate();
    // const float duration          = std::visit([](const auto &g) { return g.duration();
    // }, measurement.generator);

    // spdlog::info("Received measurement:");
    // spdlog::info("length {}, sample_rate {}, duration {} s", length, sample_rate,
    // duration);
    spdlog::info("Reference size {}", sweepData.measuredData.size());
    spdlog::info("Measured size  {}", sweepData.referenceData.size());

    // Compute sample delay between measured and reference signals
    auto measurement_delay = TimeDelay::estimate(
        sweepData.referenceData, sweepData.measuredData, sample_rate / 2, TimeDelay::PhaseDifference
    );
    // spdlog::info("estimated delay: {} cc / {} phat",
    //              TimeDelay::estimate(measurement.reference_signal,
    //              measurement.measured_signal, 0.5 * sample_rate,
    //                                  TimeDelay::CrossCorrelation),
    //              TimeDelay::estimate(measurement.reference_signal,
    //              measurement.measured_signal, 0.5 * sample_rate,
    //                                  TimeDelay::PhaseDifference));
    spdlog::info(
        "Estimated delay: {} samples / {} ms", measurement_delay,
        (1000.0 * measurement_delay) / sample_rate
    );

    /*

    // measurement.reference_signal.resize(full_length, 0);
    // Shifts measured signals according to measurement_delay
    measurement.measured_signal.erase(
        measurement.measured_signal.begin(),
        std::next(measurement.measured_signal.begin(), static_cast<int>(measurement_delay) - 1)
    );
    measurement.measured_signal.resize(measurement.reference_signal.size(), 0);

    spdlog::info("shifted reference size {}", measurement.reference_signal.size());
    spdlog::info("shifted measured size  {}", measurement.measured_signal.size());

    // Fx is the dft of x
    auto Fy = Fft::r2c(measurement.measured_signal);
    auto Fx = Fft::r2c(measurement.reference_signal);
    spdlog::info("Fy::size {}", Fy.size());
    spdlog::info("Fx::size {}", Fx.size());

    std::vector<std::complex<float>> Fh(Fy.size());
    std::transform(
        Fy.begin(), Fy.end(), Fx.begin(), Fh.begin(),
        [](const auto &Fy, const auto &Fx) { return Fy / Fx; }
    );

    // Impulse response
    auto h = Fft::c2r(Fh);
    spdlog::info("h::size {}", h.size());

    // quickPlot("reference_signal", measurement.reference_signal);
    // quickPlot("measured_signal", measurement.measured_signal);
    // quickPlot("impulse response", h, 1.f / sample_rate);

    auto reference_ft = Fft::r2c(measurement.reference_signal);
    auto measured_ft  = Fft::r2c(measurement.measured_signal);

    const auto n = measured_ft.size();

    constexpr float sensitivitydB = 4.6889F;

    std::vector<float> input_spl_ft(n);

    std::transform(
        std::execution::par, measured_ft.begin(), measured_ft.end(), input_spl_ft.begin(),
        [&](const auto &a) {
        const float normalized_magnitude = std::abs(a) / (2 * n + 1);
        const float dbfs                 = 20 * std::log10(normalized_magnitude);
        return 120 + dbfs - sensitivitydB;
    }
    );

    // i = 0;
    // for (auto d : amplitudes_fft) {
    //   double       f                    = i++ * 2 * static_cast<double>(sampleRate) /
    //   n; const double normalized_magnitude = std::abs(d) / n; const double dbfs = 20 *
    //   std::log10(normalized_magnitude);
    //   // const double spl                  = 120 + dbfs - sensitivitydB;
    //   const double spl = dbfs;

    //   data_f.push_back(f);
    //   data_spl.push_back(spl);
    // }

    int octave = 6;

    std::vector<float> input_smoothed_spl_f_ft(n), input_smoothed_spl_ft(n);

    for(std::size_t k = 0; k < input_spl_ft.size(); k++) {
        const float f = k * 2 * static_cast<float>(sample_rate) / n;

        if(f > frequencyEnd) { break; }

        if(f <= 100) {
            octave = 48;
        } else if(f <= 1'000) {
            octave = static_cast<int>(
                48.F - (f - 100.F) / 900.F * 42.F
            ); // from 48 at 100 Hz to 6 at 1000 Hz
        } else if(f >= 10'000) {
            octave = 3;
        }
        const float a_fac = std::pow(2.f, -1.f / (2.f * octave));
        const float b_fac = std::pow(2.f, 1.f / (2.f * octave));
        const int32_t ak  = static_cast<int32_t>(std::round(static_cast<float>(k) * a_fac));
        const int32_t bk  = static_cast<int32_t>(std::round(static_cast<float>(k) * b_fac));
        const float m     = 1.f / (bk - ak + 1.f);

        const float sum = std::transform_reduce(
            std::execution::par, std::next(input_spl_ft.begin(), ak),
            std::min(std::next(input_spl_ft.begin(), 1 + bk), input_spl_ft.end()),
            std::next(input_spl_ft.begin(), ak), 0.f
        );
        input_smoothed_spl_f_ft.push_back(f);
        input_smoothed_spl_ft.push_back(std::sqrt(m * sum));
    }

    std::vector<float> input_f_ft(n);
    std::generate(input_f_ft.begin(), input_f_ft.end(), [&, k = 0]() mutable {
        return k++ * 2 * static_cast<float>(sample_rate) / n;
    });

    auto graph = ui_->plotResponse->addGraph();
    graph->setData(
        QVector<double>{input_smoothed_spl_f_ft.begin(), input_smoothed_spl_f_ft.end()},
        QVector<double>{input_smoothed_spl_ft.begin(), input_smoothed_spl_ft.end()}, true
    );
    graph->rescaleAxes();
    // ui_->plot->graph(1)->setData(QVector<double>::fromStdVector(data_f),
    //                             QVector<double>::fromStdVector(data_spl), true);
    ui_->plotResponse->graph(1)->rescaleAxes();
    ui_->plotResponse->xAxis->setRange(frequencyBegin, frequencyEnd);
    ui_->plotResponse->replot();

    std::vector<std::complex<float>> impulse_ft(measured_ft.size());

    std::transform(
        std::execution::seq, measured_ft.begin(), measured_ft.end(), reference_ft.begin(),
        impulse_ft.begin(),
        [](const auto &measured, const auto &reference) { return measured / reference; }
    );
    auto impulse_ts = Fft::c2r(impulse_ft);

    std::vector<float> impulse_t_ts(impulse_ts.size());
    std::generate(impulse_t_ts.begin(), impulse_t_ts.end(), [&, k = 0]() mutable {
        return k++ / static_cast<float>(sample_rate);
    });

    ui_->plotResponse->clearGraphs();
    ui_->plotResponse->addGraph();
    ui_->plotResponse->graph(0)->addData(
        QVector<double>{impulse_t_ts.begin(), impulse_t_ts.end()},
        QVector<double>{impulse_ts.begin(), impulse_ts.end()}
    );
    ui_->plotResponse->graph(0)->rescaleAxes();
    ui_->plotResponse->replot();
    */

    // Restore ui
    ui_->grpMeasParams->setEnabled(true);
}

// void MainWindow::handleRtaData()
// {
//     auto data_arr = audio_.get_latest_rta_data_();
//     // std::vector<float> data(data_arr.begin(), data_arr.end());
//     auto data_ft  = Fft::r2c({data_arr.begin(), data_arr.end()});
//     auto n       = data_ft.size();

//     constexpr float sensitivitydB = 4.6889F;

//     std::vector<float> input_spl_ft(n);

//     std::transform(
//         std::execution::par, data_ft.begin(), data_ft.end(), input_spl_ft.begin(),
//         [&](const auto &a) {
//         const float normalized_magnitude = std::abs(a) / (2 * n + 1);
//         const float dbfs                 = 20 * std::log10(normalized_magnitude);
//         return 120 + dbfs - sensitivitydB;
//     }
//     );

//     // quickPlot_.clearGraphs();
//     // quickPlot_.xAxis->setScaleType(QCPAxis::stLogarithmic);
//     QSharedPointer<QCPAxisTickerLog> log_ticker(new QCPAxisTickerLog);
//     // quickPlot_.xAxis->setTicker(log_ticker);
//     // quickPlot("rta", input_spl_ft);
// }

void MainWindow::setupPlotForLogLine(QCustomPlot &plot)
{
    plot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    // Legend
    plot.legend->setVisible(true);
    plot.legend->setBrush(QBrush{
        QColor{255, 255, 255, 150}
    });
    // Axes
    plot.axisRect()->setupFullAxesBox();
    plot.xAxis->grid()->setSubGridVisible(true);
    plot.xAxis->setScaleType(QCPAxis::stLogarithmic);
    plot.xAxis->setTicker(QSharedPointer<QCPAxisTickerLog>{new QCPAxisTickerLog});
    connect(
        plot.xAxis, qOverload<const QCPRange &>(&QCPAxis::rangeChanged), plot.xAxis2,
        qOverload<const QCPRange &>(&QCPAxis::setRange)
    );
    connect(
        plot.yAxis, qOverload<const QCPRange &>(&QCPAxis::rangeChanged), plot.yAxis2,
        qOverload<const QCPRange &>(&QCPAxis::setRange)
    );
}

void MainWindow::startMeasurement()
{
    // Get measurement parameters
    const auto frequencyBegin = ui_->eBeginFrequency->value();
    const auto frequencyEnd   = ui_->eEndFrequency->value();
    const auto length      = ui_->eLength->currentData().toUInt();
    const auto sampleRate     = ui_->eSampleRate->currentData().toULongLong();
    const auto volumeDBFS     = static_cast<float>(ui_->eVolumeDBFS->value());
    // const auto duration    = static_cast<double>(length) / sample_rate;

    // Prevent modifications while measuring
    ui_->grpMeasParams->setEnabled(false);

    const uint32_t lengthSilence = 0.5 * sampleRate; // 0.5s of silence

    audio_.startSweep(frequencyBegin, frequencyEnd, length, lengthSilence, sampleRate, volumeDBFS);
}

void MainWindow::updateMeasurementDuration()
{
    const float frequencyBegin = ui_->eBeginFrequency->value();
    const float frequencyEnd   = ui_->eEndFrequency->value();
    const uint32_t sampleRate  = ui_->eSampleRate->currentData().toUInt();
    const uint32_t length      = ui_->eLength->currentData().toUInt();

    const uint32_t lengthSilence = 0.5 * sampleRate; // 0.5s of silence

    Generators::SynchronizedSweptSine generator{frequencyBegin, frequencyEnd, length,
                                                lengthSilence,  sampleRate,   0.25};

    spdlog::info("Duration {:.2f}s / {:.2f}s", generator.duration(false), generator.duration());

    ui_->lvDuration->setText(QString::fromStdString(fmt::format("{:.1f} s", generator.duration())));
}

void MainWindow::updateStartFrequency(int frequencyInitial)
{
    auto frequencyFinal = ui_->eEndFrequency->value();
    if(frequencyFinal <= frequencyInitial) { ui_->eEndFrequency->setValue(frequencyInitial + 1); }

    ui_->plotResponse->xAxis->setRange(frequencyInitial, ui_->eEndFrequency->value());
    ui_->plotResponse->replot();
}

void MainWindow::updateEndFrequency(int frequencyFinal)
{
    auto frequencyInitial = ui_->eBeginFrequency->value();
    if(frequencyInitial >= frequencyFinal) { ui_->eBeginFrequency->setValue(frequencyFinal - 1); }

    ui_->plotResponse->xAxis->setRange(ui_->eBeginFrequency->value(), frequencyFinal);
    ui_->plotResponse->replot();
}

} // namespace Audio

// void Audio::MainWindow::toggleRTA() { audio_.startRTA(); }

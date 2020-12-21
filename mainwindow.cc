#include "mainwindow.hh"
#include "./ui_mainwindow.h"
#include "generators/generators.hh"
#include "timedelay.hh"

#include <QDebug>
#include <QtAwesome/QtAwesome.h>
#include <fft.hh>
#include <fftw3.h>
#include <range/v3/action/drop.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <concepts>
#include <execution>
#include <numbers>

namespace Audio {

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    awesome_ = new QtAwesome(this);
    awesome_->initFontAwesome();

    ui->setupUi(this);
    setupUi();

    connect(&audio_, &AudioIO::audio_finished, this, &MainWindow::handle_finished);
    connect(&audio_, &AudioIO::on_rta_data, this, &MainWindow::handle_rta_data);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::setupUi() {
    {
        quick_plot_.addGraph();
        quick_plot_.addGraph();
        quick_plot_.graph(0)->setPen({Qt::blue});
        quick_plot_.graph(1)->setPen({Qt::red});
        quick_plot_.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
        quick_plot_.xAxis->grid()->setSubGridVisible(true);
        quick_plot_.axisRect()->setupFullAxesBox();
        connect(quick_plot_.xAxis, qOverload<const QCPRange &>(&QCPAxis::rangeChanged), quick_plot_.xAxis2,
                qOverload<const QCPRange &>(&QCPAxis::setRange));
        connect(quick_plot_.yAxis, qOverload<const QCPRange &>(&QCPAxis::rangeChanged), quick_plot_.yAxis2,
                qOverload<const QCPRange &>(&QCPAxis::setRange));
    }

    {
        ui->plot->addGraph();
        ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
        ui->plot->xAxis->grid()->setSubGridVisible(true);
        ui->plot->xAxis->setScaleType(QCPAxis::stLogarithmic);
        ui->plot->axisRect()->setupFullAxesBox();
        connect(ui->plot->xAxis, qOverload<const QCPRange &>(&QCPAxis::rangeChanged), ui->plot->xAxis2,
                qOverload<const QCPRange &>(&QCPAxis::setRange));
        connect(ui->plot->yAxis, qOverload<const QCPRange &>(&QCPAxis::rangeChanged), ui->plot->yAxis2,
                qOverload<const QCPRange &>(&QCPAxis::setRange));
        ui->plot->xAxis->setNumberPrecision(0);
        ui->plot->xAxis->setNumberFormat("f");
        QSharedPointer<QCPAxisTickerLog> log_ticker(new QCPAxisTickerLog);
        ui->plot->xAxis->setTicker(log_ticker);

        ui->plot2->addGraph();
        ui->plot2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
        ui->plot2->xAxis->grid()->setSubGridVisible(true);
    }

    // Supported sample rates combo box
    {
        for (auto sampleRate : audio_.supportedSampleRates()) {
            ui->eSampleRate->addItem(QString::number(sampleRate), sampleRate);
        }
        ui->eSampleRate->setCurrentText("48000");

        // Length
        ui->eLength->addItem("128k", (1u << 17u));
        ui->eLength->addItem("256k", (1u << 18u));
        ui->eLength->addItem("512k", (1u << 19u));
        ui->eLength->addItem("1M", (1u << 20u));
        ui->eLength->setCurrentIndex(1);
    }

    // Setup colormap
    {
        ui->plotSpectrogram->axisRect()->setupFullAxesBox(true);
        // ui->plotSpectrogram->xAxis->setScaleType(QCPAxis::stLogarithmic);
        // QSharedPointer<QCPAxisTickerLog> log_ticker(new QCPAxisTickerLog);
        // ui->plotSpectrogram->xAxis->setTicker(log_ticker);
        ui->plotSpectrogram->xAxis->setLabel("Frequency [Hz]");
        ui->plotSpectrogram->yAxis->setLabel("Time [s]");
        color_map_ = new QCPColorMap(ui->plotSpectrogram->xAxis, ui->plotSpectrogram->yAxis);
        color_map_->data()->setSize(200, 200);
        color_map_->data()->setRange(QCPRange(-4, 4), QCPRange(-4, 4)); // [20; 20000] Hz by [0; 10] s
        color_map_->setGradient(QCPColorGradient::gpNight);

        double x, y, z;
        for (int xIndex = 0; xIndex < 200; ++xIndex) {
            for (int yIndex = 0; yIndex < 200; ++yIndex) {
                color_map_->data()->cellToCoord(xIndex, yIndex, &x, &y);
                double r = 3 * qSqrt(x * x + y * y) + 1e-2;
                z        = 2 * x *
                    (qCos(r + 2) / r -
                     qSin(r + 2) / r); // the B field strength of dipole radiation (modulo physical constants)
                color_map_->data()->setCell(xIndex, yIndex, z);
            }
        }

        color_map_->rescaleDataRange();
        ui->plotSpectrogram->rescaleAxes();
    }

    // Connections
    connect(ui->eSampleRate, qOverload<int>(&QComboBox::currentIndexChanged), this,
            &MainWindow::updateMeasurementDuration);
    connect(ui->eLength, qOverload<int>(&QComboBox::currentIndexChanged), this, &MainWindow::updateMeasurementDuration);

    // Update ui
    updateMeasurementDuration();
    ui->plot->xAxis->setRange(ui->eStartFreq->value(), ui->eEndFreq->value());
}

void MainWindow::quickPlot(std::string name, const std::vector<float> &signal, float scaling /* = 1*/) {
    std::vector<float> signal_t(signal.size());
    std::generate(signal_t.begin(), signal_t.end(), [&, n = 0]() mutable { return n++ * scaling; });

    auto *graph      = quick_plot_.addGraph();
    const auto index = quick_plot_.graphCount() - 1;
    graph->setPen(QColor::fromHslF(std::fmod(index * 0.618033988749895f, 1.0f), 1.0, 0.5));
    graph->setName(QString::fromStdString(name));

    graph->setData(QVector<double>(signal_t.begin(), signal_t.end()), QVector<double>(signal.begin(), signal.end()));
    quick_plot_.replot();
    quick_plot_.rescaleAxes();
    quick_plot_.show();
    quick_plot_.legend->setVisible(true);
}

void MainWindow::handle_finished() {
    quick_plot_.clearGraphs();
    using namespace ranges;

    auto measurement = audio_.getMeasurement();

    // Get measurement parameters
    const float f0                = ui->eStartFreq->value();
    const float ff                = ui->eEndFreq->value();
    const std::size_t full_length = ui->eLength->currentData().toUInt();
    const std::size_t length      = std::visit([](const auto &g) { return g.length(); }, measurement.generator);
    const std::size_t sample_rate = std::visit([](const auto &g) { return g.sample_rate(); }, measurement.generator);
    const float duration          = std::visit([](const auto &g) { return g.duration(); }, measurement.generator);

    // spdlog::info("Received measurement:");
    // spdlog::info("length {}, sample_rate {}, duration {} s", length, sample_rate, duration);
    spdlog::info("original reference size {}", measurement.reference_signal.size());
    spdlog::info("original measured size  {}", measurement.measured_signal.size());

    // Compute sample delay between measured and reference signals
    auto measurement_delay = TimeDelay::estimate(measurement.reference_signal, measurement.measured_signal,
                                                 0.5 * sample_rate, TimeDelay::PhaseDifference);
    /*spdlog::info("estimated delay: {} cc / {} phat",
                 TimeDelay::estimate(measurement.reference_signal, measurement.measured_signal, 0.5 * sample_rate,
                                     TimeDelay::CrossCorrelation),
                 TimeDelay::estimate(measurement.reference_signal, measurement.measured_signal, 0.5 * sample_rate,
                                     TimeDelay::PhaseDifference));
    */
    spdlog::info("Estimated delay: {} samples / {} ms", measurement_delay, (1000.0 * measurement_delay) / sample_rate);

    // measurement.reference_signal.resize(full_length, 0);
    // Shifts measured signals according to measurement_delay
    measurement.measured_signal.erase(measurement.measured_signal.begin(),
                                      std::next(measurement.measured_signal.begin(), measurement_delay - 1));
    measurement.measured_signal.resize(measurement.reference_signal.size(), 0);

    spdlog::info("shifted reference size {}", measurement.reference_signal.size());
    spdlog::info("shifted measured size  {}", measurement.measured_signal.size());

    // Fx is the dft of x
    auto Fy = fft::r2c(measurement.measured_signal);
    auto Fx = fft::r2c(measurement.reference_signal);
    spdlog::info("Fy::size {}", Fy.size());
    spdlog::info("Fx::size {}", Fx.size());

    std::vector<std::complex<float>> Fh(Fy.size());
    std::transform(Fy.begin(), Fy.end(), Fx.begin(), Fh.begin(),
                   [](const auto &Fy, const auto &Fx) { return Fy / Fx; });

    // Impulse response
    auto h = fft::c2r(Fh);
    spdlog::info("h::size {}", h.size());

    // quickPlot("reference_signal", measurement.reference_signal);
    // quickPlot("measured_signal", measurement.measured_signal);
    quickPlot("impulse response", h, 1.f / sample_rate);

    auto reference_ft = fft::r2c(measurement.reference_signal);
    auto measured_ft  = fft::r2c(measurement.measured_signal);

    const auto n = measured_ft.size();

    constexpr float sensitivitydB = 4.6889;

    std::vector<float> input_spl_ft(n);

    std::transform(std::execution::par, measured_ft.begin(), measured_ft.end(), input_spl_ft.begin(),
                   [&](const auto &a) {
                       const float normalized_magnitude = std::abs(a) / (2 * n + 1);
                       const float dbfs                 = 20 * std::log10(normalized_magnitude);
                       return 120 + dbfs - sensitivitydB;
                   });

    /*i = 0;
    for (auto d : amplitudes_fft) {
      double       f                    = i++ * 2 * static_cast<double>(sampleRate) / n;
      const double normalized_magnitude = std::abs(d) / n;
      const double dbfs                 = 20 * std::log10(normalized_magnitude);
      // const double spl                  = 120 + dbfs - sensitivitydB;
      const double spl = dbfs;

      data_f.push_back(f);
      data_spl.push_back(spl);
    }*/

    int octave = 6;

    std::vector<float> input_smoothed_spl_f_ft(n), input_smoothed_spl_ft(n);

    for (std::size_t k = 0; k < input_spl_ft.size(); k++) {
        const float f = k * 2 * static_cast<float>(sample_rate) / n;

        if (f > ff) break;

        if (f <= 100)
            octave = 48;
        else if (f <= 1000)
            octave = 48 - (f - 100) / 900 * 42; // from 48 at 100 Hz to 6 at 1000 Hz
        else if (f >= 10000)
            octave = 3;
        const float a_fac    = std::pow(2.f, -1.f / (2.f * octave));
        const float b_fac    = std::pow(2.f, 1.f / (2.f * octave));
        const std::size_t ak = std::round(k * a_fac);
        const std::size_t bk = std::round(k * b_fac);
        const float m        = 1.f / (bk - ak + 1.f);

        const float sum = std::transform_reduce(std::execution::par, input_spl_ft.begin() + ak,
                                                std::min(input_spl_ft.begin() + 1 + bk, input_spl_ft.end()),
                                                input_spl_ft.begin() + ak, 0.f);
        input_smoothed_spl_f_ft.push_back(f);
        input_smoothed_spl_ft.push_back(std::sqrt(m * sum));
    }

    std::vector<float> input_f_ft(n);
    std::generate(input_f_ft.begin(), input_f_ft.end(),
                  [&, k = 0]() mutable { return k++ * 2 * static_cast<float>(sample_rate) / n; });

    auto graph = ui->plot->addGraph();
    graph->setData(QVector<double>{input_smoothed_spl_f_ft.begin(), input_smoothed_spl_f_ft.end()},
                   QVector<double>{input_smoothed_spl_ft.begin(), input_smoothed_spl_ft.end()}, true);
    graph->rescaleAxes();
    /*ui->plot->graph(1)->setData(QVector<double>::fromStdVector(data_f),
                                QVector<double>::fromStdVector(data_spl), true);*/
    ui->plot->graph(1)->rescaleAxes();
    ui->plot->xAxis->setRange(f0, ff);
    ui->plot->replot();

    std::vector<std::complex<float>> impulse_ft(measured_ft.size());

    std::transform(std::execution::seq, measured_ft.begin(), measured_ft.end(), reference_ft.begin(),
                   impulse_ft.begin(),
                   [](const auto &measured, const auto &reference) { return measured / reference; });
    auto impulse_ts = fft::c2r(impulse_ft);

    std::vector<float> impulse_t_ts(impulse_ts.size());
    std::generate(impulse_t_ts.begin(), impulse_t_ts.end(),
                  [&, k = 0]() mutable { return k++ / static_cast<float>(sample_rate); });

    ui->plot2->clearGraphs();
    ui->plot2->addGraph();
    ui->plot2->graph(0)->addData(QVector<double>{impulse_t_ts.begin(), impulse_t_ts.end()},
                                 QVector<double>{impulse_ts.begin(), impulse_ts.end()});
    ui->plot2->graph(0)->rescaleAxes();
    ui->plot2->replot();

    // Restore ui
    ui->grpMeasParams->setEnabled(true);
}

void MainWindow::handle_rta_data() {
    auto data_arr = audio_.get_latest_rta_data_();
    std::vector<float> data(data_arr.begin(), data_arr.end());
    auto data_ft = fft::r2c(data);
    auto n       = data_ft.size();

    constexpr float sensitivitydB = 4.6889;

    std::vector<float> input_spl_ft(n);

    std::transform(std::execution::par, data_ft.begin(), data_ft.end(), input_spl_ft.begin(), [&](const auto &a) {
        const float normalized_magnitude = std::abs(a) / (2 * n + 1);
        const float dbfs                 = 20 * std::log10(normalized_magnitude);
        return 120 + dbfs - sensitivitydB;
    });

    quick_plot_.clearGraphs();
    quick_plot_.xAxis->setScaleType(QCPAxis::stLogarithmic);
    QSharedPointer<QCPAxisTickerLog> log_ticker(new QCPAxisTickerLog);
    quick_plot_.xAxis->setTicker(log_ticker);
    quickPlot("rta", input_spl_ft);
}

void MainWindow::on_bMeasure_clicked() {
    // Get measurement parameters
    const auto f0          = ui->eStartFreq->value();
    const auto ff          = ui->eEndFreq->value();
    const auto length      = ui->eLength->currentData().toUInt();
    const auto sample_rate = ui->eSampleRate->currentData().toInt();
    const auto volume_DBFS = ui->eVolumeDBFS->value();
    const auto duration    = static_cast<double>(length) / sample_rate;

    // Disable further editing
    ui->grpMeasParams->setEnabled(false);

    auto generator =
        Generator::SynchronizedSweptSine(ui->eStartFreq->value(), ui->eEndFreq->value(), length, sample_rate);

    // mAudio.start(generator, volume_DBFS);
    audio_.startSweep(f0, ff, length, sample_rate, volume_DBFS);
}

void MainWindow::updateMeasurementDuration() {
    const auto sample_rate = ui->eSampleRate->currentData().toInt();
    const auto length      = ui->eLength->currentData().toUInt();

    auto generator =
        Generator::SynchronizedSweptSine(ui->eStartFreq->value(), ui->eEndFreq->value(), length, sample_rate);

    spdlog::info("l1: {} s", static_cast<float>(length) / sample_rate);
    spdlog::info("l2: {} s", generator.duration());

    ui->lvDuration->setText(QString::number(generator.duration(), 'f', 1) + " s");
}

void MainWindow::on_eStartFreq_valueChanged(int f0) {
    auto ff = ui->eEndFreq->value();
    if (ff <= f0) { ui->eEndFreq->setValue(f0 + 1); }

    ui->plot->xAxis->setRange(f0, ff);
}

void MainWindow::on_eEndFreq_valueChanged(int ff) {
    auto f0 = ui->eStartFreq->value();
    if (f0 >= ff) { ui->eStartFreq->setValue(ff - 1); }

    ui->plot->xAxis->setRange(f0, ff);
}

} // namespace Audio

void Audio::MainWindow::on_bToggleSpectrogram_clicked() { audio_.startRTA(); }

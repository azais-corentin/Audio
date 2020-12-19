#include "mainwindow.hh"
#include "./ui_mainwindow.h"
#include "generators/generators.hh"
#include "interface/QtAwesome/QtAwesome.h"
#include "timedelay.hh"

#include <fftw3.h>
#include <spdlog/spdlog.h>
#include <QDebug>
#include <fft.hh>
#include <range/v3/action/drop.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/drop.hpp>
#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/zip.hpp>

#include <algorithm>
#include <concepts>
#include <execution>
#include <numbers>

namespace Audio {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  mAwesome = new QtAwesome(this);
  mAwesome->initFontAwesome();

  ui->setupUi(this);
  setupUi();

  connect(&mAudio, &AudioIO::audioFinished, this, &MainWindow::handleFinished);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::setupUi()
{
  mQuickPlot.addGraph();
  mQuickPlot.addGraph();
  mQuickPlot.graph(0)->setPen({Qt::blue});
  mQuickPlot.graph(1)->setPen({Qt::red});
  mQuickPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  mQuickPlot.xAxis->grid()->setSubGridVisible(true);
  mQuickPlot.axisRect()->setupFullAxesBox();
  connect(mQuickPlot.xAxis, qOverload<const QCPRange&>(&QCPAxis::rangeChanged),
          mQuickPlot.xAxis2, qOverload<const QCPRange&>(&QCPAxis::setRange));
  connect(mQuickPlot.yAxis, qOverload<const QCPRange&>(&QCPAxis::rangeChanged),
          mQuickPlot.yAxis2, qOverload<const QCPRange&>(&QCPAxis::setRange));

  ui->plot->addGraph();
  ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  ui->plot->xAxis->grid()->setSubGridVisible(true);
  ui->plot->xAxis->setScaleType(QCPAxis::stLogarithmic);
  ui->plot->axisRect()->setupFullAxesBox();
  connect(ui->plot->xAxis, qOverload<const QCPRange&>(&QCPAxis::rangeChanged),
          ui->plot->xAxis2, qOverload<const QCPRange&>(&QCPAxis::setRange));
  connect(ui->plot->yAxis, qOverload<const QCPRange&>(&QCPAxis::rangeChanged),
          ui->plot->yAxis2, qOverload<const QCPRange&>(&QCPAxis::setRange));
  ui->plot->xAxis->setNumberPrecision(0);
  ui->plot->xAxis->setNumberFormat("f");
  QSharedPointer<QCPAxisTickerLog> logTicker(new QCPAxisTickerLog);
  ui->plot->xAxis->setTicker(logTicker);

  ui->plot2->addGraph();
  ui->plot2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  ui->plot2->xAxis->grid()->setSubGridVisible(true);

  // Supported sample rates combo box
  for (auto sampleRate : mAudio.supportedSampleRates()) {
    ui->eSampleRate->addItem(QString::number(sampleRate), sampleRate);
  }
  ui->eSampleRate->setCurrentText("48000");

  // Length
  ui->eLength->addItem("128k", (1u << 17u));
  ui->eLength->addItem("256k", (1u << 18u));
  ui->eLength->addItem("512k", (1u << 19u));
  ui->eLength->addItem("1M", (1u << 20u));
  ui->eLength->setCurrentIndex(1);

  // Connections
  connect(ui->eSampleRate, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateMeasurementDuration);
  connect(ui->eLength, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateMeasurementDuration);

  // Update ui
  updateMeasurementDuration();
  ui->plot->xAxis->setRange(ui->eStartFreq->value(), ui->eEndFreq->value());
}

void MainWindow::handleFinished()
{
  using namespace ranges;

  auto measurement = mAudio.getMeasurement();

  // Get measurement parameters
  const float       f0          = ui->eStartFreq->value();
  const float       ff          = ui->eEndFreq->value();
  const std::size_t full_length = ui->eLength->currentData().toUInt();
  const std::size_t length =
      std::visit([](const auto& g) { return g.length(); }, measurement.generator);
  const std::size_t sample_rate =
      std::visit([](const auto& g) { return g.sample_rate(); }, measurement.generator);
  const float duration =
      std::visit([](const auto& g) { return g.duration(); }, measurement.generator);

  spdlog::info("Received measurement:");
  spdlog::info("length {}, sample_rate {}, duration {} s", length, sample_rate, duration);
  spdlog::info("reference: length {}", measurement.reference_signal.size());
  spdlog::info("measured: length {}", measurement.measured_signal.size());

  // Compute sample delay between measured and reference signals
  auto sample_delay =
      TimeDelay::estimate(measurement.reference_signal, measurement.measured_signal,
                          0.5 * sample_rate, TimeDelay::CrossCorrelation);
  if (sample_delay < 0)
    sample_delay = 0;
  spdlog::info("Estimated delay: {} samples / {} ms", sample_delay,
               (1000.0 * sample_delay) / sample_rate);

  // Shifts both reference and measured signals according to sample
  measurement.reference_signal.resize(full_length);
  // Remove the first n samples
  measurement.measured_signal.erase(
      measurement.measured_signal.begin(),
      std::next(measurement.measured_signal.begin(), sample_delay));
  measurement.measured_signal.resize(full_length);

  auto Fy = fft::r2c(measurement.measured_signal);
  auto xtilde =
      std::get<Generator::SynchronizedSweptSine>(measurement.generator).xtilde();

  spdlog::info("xtilde::size {}", xtilde.size());
  spdlog::info("Fy::size {}", Fy.size());

  std::vector<std::complex<float>> H(Fy.size());
  std::transform(Fy.begin(), Fy.end(), xtilde.begin(), H.begin(),
                 [](const auto& Fy, const auto& xtilde) { return Fy * xtilde; });

  auto h = fft::c2r(H);

  spdlog::info("h::size {}", h.size());

  std::vector<float> signal_t(h.size());
  std::generate(signal_t.begin(), signal_t.end(),
                [&, n = -1.f / sample_rate]() mutable { return n += 1.f / sample_rate; });

  mQuickPlot.graph(0)->data().clear();
  mQuickPlot.graph(0)->addData(QVector<double>{signal_t.begin(), signal_t.end()},
                               QVector<double>{h.begin(), h.end()});
  mQuickPlot.graph(1)->data().clear();
  /*mQuickPlot.graph(1)->addData(QVector<double>{signal_t.begin(), signal_t.end()},
                               QVector<double>{measurement.measured_signal.begin(),
                                               measurement.measured_signal.end()});*/
  mQuickPlot.rescaleAxes();
  mQuickPlot.replot();
  mQuickPlot.show();

  spdlog::info("shifted reference: length {}", measurement.reference_signal.size());
  spdlog::info("shifted measured: length {}", measurement.measured_signal.size());

  auto reference_ft = fft::r2c(measurement.reference_signal);
  auto measured_ft  = fft::r2c(measurement.measured_signal);

  const auto n = measured_ft.size();

  constexpr float sensitivitydB = 4.6889;

  std::vector<float> input_spl_ft(n);

  std::transform(std::execution::par, measured_ft.begin(), measured_ft.end(),
                 input_spl_ft.begin(), [&](const auto& a) {
                   const float normalized_magnitude = std::abs(a) / n;
                   const float dbfs = 20 * std::log10(normalized_magnitude);
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

    if (f > ff)
      break;

    if (f <= 100)
      octave = 48;
    else if (f <= 1000)
      octave = 48 - (f - 100) / 900 * 42;  // from 48 at 100 Hz to 6 at 1000 Hz
    else if (f >= 10000)
      octave = 3;
    const float       a_fac = std::pow(2.f, -1.f / (2.f * octave));
    const float       b_fac = std::pow(2.f, 1.f / (2.f * octave));
    const std::size_t ak    = std::round(k * a_fac);
    const std::size_t bk    = std::round(k * b_fac);
    const float       m     = 1.f / (bk - ak + 1.f);

    const float sum =
        std::transform_reduce(std::execution::par, input_spl_ft.begin() + ak,
                              std::min(input_spl_ft.begin() + 1 + bk, input_spl_ft.end()),
                              input_spl_ft.begin() + ak, 0.f);
    input_smoothed_spl_f_ft.push_back(f);
    input_smoothed_spl_ft.push_back(std::sqrt(m * sum));
  }

  std::vector<float> input_f_ft(n);
  std::generate(input_f_ft.begin(), input_f_ft.end(), [&, k = 0]() mutable {
    return k++ * 2 * static_cast<float>(sample_rate) / n;
  });

  auto graph = ui->plot->addGraph();
  graph->setData(
      QVector<double>{input_smoothed_spl_f_ft.begin(), input_smoothed_spl_f_ft.end()},
      QVector<double>{input_smoothed_spl_ft.begin(), input_smoothed_spl_ft.end()}, true);
  graph->rescaleAxes();
  /*ui->plot->graph(1)->setData(QVector<double>::fromStdVector(data_f),
                              QVector<double>::fromStdVector(data_spl), true);*/
  ui->plot->graph(1)->rescaleAxes();
  ui->plot->xAxis->setRange(f0, ff);
  ui->plot->replot();

  std::vector<std::complex<float>> impulse_ft(measured_ft.size());

  std::transform(
      std::execution::seq, measured_ft.begin(), measured_ft.end(), reference_ft.begin(),
      impulse_ft.begin(),
      [](const auto& measured, const auto& reference) { return measured / reference; });
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

void MainWindow::on_bMeasure_clicked()
{
  // Get measurement parameters
  const auto f0         = ui->eStartFreq->value();
  const auto ff         = ui->eEndFreq->value();
  const auto length     = ui->eLength->currentData().toUInt();
  const auto sampleRate = ui->eSampleRate->currentData().toInt();
  const auto volumeDBFS = ui->eVolumeDBFS->value();
  const auto duration   = static_cast<double>(length) / sampleRate;

  // Disable further editing
  ui->grpMeasParams->setEnabled(false);

  mAudio.startSweep(f0, ff, length, sampleRate, volumeDBFS);
}

void MainWindow::updateMeasurementDuration()
{
  const auto sample_rate = ui->eSampleRate->currentData().toInt();
  const auto length      = ui->eLength->currentData().toUInt();

  auto generator = Generator::SynchronizedSweptSine(
      ui->eStartFreq->value(), ui->eEndFreq->value(), length, sample_rate);

  spdlog::info("l1: {} s", static_cast<float>(length) / sample_rate);
  spdlog::info("l2: {} s", generator.duration());

  ui->lvDuration->setText(QString::number(generator.duration(), 'f', 1) + " s");
}

void MainWindow::on_eStartFreq_valueChanged(int f0)
{
  auto ff = ui->eEndFreq->value();
  if (ff <= f0) {
    ui->eEndFreq->setValue(f0 + 1);
  }

  ui->plot->xAxis->setRange(f0, ff);
}

void MainWindow::on_eEndFreq_valueChanged(int ff)
{
  auto f0 = ui->eStartFreq->value();
  if (f0 >= ff) {
    ui->eStartFreq->setValue(ff - 1);
  }

  ui->plot->xAxis->setRange(f0, ff);
}

}  // namespace Audio

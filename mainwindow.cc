#include "mainwindow.hh"
#include "./ui_mainwindow.h"

#include <algorithm>
#include <execution>
#include <numbers>

#include <fftw3.h>
#include <spdlog/spdlog.h>
#include <QDebug>
#include <fft.hh>
#include <range/v3/view/zip.hpp>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  setupUi();

  connect(&mAudio, &AudioIO::audioFinished, []() { spdlog::info("Audio finished!"); });

  mAudio.startSweep(20, 20000, 1 << 15, 192000, -12);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::setupUi()
{
  mQuickPlot.addGraph();
  mQuickPlot.setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  mQuickPlot.xAxis->grid()->setSubGridVisible(true);
  mQuickPlot.axisRect()->setupFullAxesBox();
  connect(mQuickPlot.xAxis, qOverload<const QCPRange&>(&QCPAxis::rangeChanged), mQuickPlot.xAxis2,
          qOverload<const QCPRange&>(&QCPAxis::setRange));
  connect(mQuickPlot.yAxis, qOverload<const QCPRange&>(&QCPAxis::rangeChanged), mQuickPlot.yAxis2,
          qOverload<const QCPRange&>(&QCPAxis::setRange));

  ui->plot->addGraph();
  ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  ui->plot->xAxis->grid()->setSubGridVisible(true);
  ui->plot->xAxis->setScaleType(QCPAxis::stLogarithmic);
  ui->plot->axisRect()->setupFullAxesBox();
  connect(ui->plot->xAxis, qOverload<const QCPRange&>(&QCPAxis::rangeChanged), ui->plot->xAxis2,
          qOverload<const QCPRange&>(&QCPAxis::setRange));
  connect(ui->plot->yAxis, qOverload<const QCPRange&>(&QCPAxis::rangeChanged), ui->plot->yAxis2,
          qOverload<const QCPRange&>(&QCPAxis::setRange));
  ui->plot->xAxis->setNumberPrecision(0);
  ui->plot->xAxis->setNumberFormat("f");
  QSharedPointer<QCPAxisTickerLog> logTicker(new QCPAxisTickerLog);
  ui->plot->xAxis->setTicker(logTicker);

  ui->plot2->addGraph();
  ui->plot2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  ui->plot2->xAxis->grid()->setSubGridVisible(true);

  // Supported sample rates combo box
  const auto outputSR = QAudioDeviceInfo::defaultOutputDevice().supportedSampleRates();
  const auto inputSR  = QAudioDeviceInfo::defaultInputDevice().supportedSampleRates();
  const auto supportedSRset =
      QSet<int>{outputSR.begin(), outputSR.end()}.intersect({inputSR.begin(), inputSR.end()});
  auto supportedSR = QList<int>{supportedSRset.begin(), supportedSRset.end()};
  std::sort(supportedSR.begin(), supportedSR.end());

  for (auto sampleRate : supportedSR) {
    ui->eSampleRate->addItem(QString::number(sampleRate), sampleRate);
  }
  ui->eSampleRate->setCurrentIndex(ui->eSampleRate->count() - 1);

  // Length
  ui->eLength->addItem("14", (1u << 14u));
  ui->eLength->addItem("15", (1u << 15u));
  ui->eLength->addItem("16", (1u << 16u));
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
  connect(&mBufferInput, &QBuffer::readyRead, this, &MainWindow::handleRecordedData);

  // Update ui
  updateMeasurementDuration();
  ui->plot->xAxis->setRange(ui->eStartFreq->value(), ui->eEndFreq->value());
}

void MainWindow::handleFinished()
{
  // Stop measurement
  mAudioOutput->stop();
  mAudioInput->stop();

  // Get measurement parameters
  const double f0         = ui->eStartFreq->value();
  const double ff         = ui->eEndFreq->value();
  const auto   length     = ui->eLength->currentData().toUInt();
  const auto   sampleRate = ui->eSampleRate->currentData().toInt();
  const double duration   = static_cast<double>(length) / sampleRate;

  // Configure input buffer & stream
  mBufferInput.close();
  mBufferInput.open(QIODevice::ReadOnly | QIODevice::Unbuffered);
  qDebug() << "size:" << mBufferInput.size() / mBytesPerSample;
  mBufferInput.seek(mBufferInput.size() - length * mBytesPerSample);
  QDataStream input(&mBufferInput);
  input.setByteOrder(QDataStream::LittleEndian);

  uint32_t            i = 0;
  std::vector<double> input_ts;
  while (!input.atEnd()) {
    int16_t sample_amplitude;
    input >> sample_amplitude;
    double amplitude = sample_amplitude / mSampleFactor;

    input_ts.push_back(amplitude);
    i++;
  }
  qDebug() << "Samples received:" << i;

  auto       input_ft = fft::r2c(input_ts);
  const auto n        = input_ft.size();
  qDebug() << "FFT size:" << n;

  constexpr double sensitivitydB = 4.6889;

  std::vector<double> input_spl_ft(n);

  std::transform(std::execution::par, input_ft.begin(), input_ft.end(), input_spl_ft.begin(),
                 [&](const auto& a) {
                   const double normalized_magnitude = std::abs(a) / n;
                   const double dbfs                 = 20 * std::log10(normalized_magnitude);
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

  std::vector<double> input_smoothed_spl_f_ft(n), input_smoothed_spl_ft(n);

  for (std::size_t k = 0; k < input_spl_ft.size(); k++) {
    const double f = k * 2 * static_cast<double>(sampleRate) / n;

    if (f > ff)
      break;

    if (f <= 100)
      octave = 48;
    else if (f <= 1000)
      octave = 48 - (f - 100) / 900 * 42;  // from 48 at 100 Hz to 6 at 1000 Hz
    else if (f >= 10000)
      octave = 3;
    const double      a_fac = std::pow(2., -1. / (2. * octave));
    const double      b_fac = std::pow(2., 1. / (2. * octave));
    const std::size_t ak    = std::round(k * a_fac);
    const std::size_t bk    = std::round(k * b_fac);
    const double      m     = 1. / (bk - ak + 1);

    /*const double sum = std::inner_product(
        data_spl.begin() + ak, std::min(data_spl.begin() + 1 + bk, data_spl.end()),
        data_spl.begin() + ak, 0);*/
    const double sum =
        std::transform_reduce(std::execution::par, input_spl_ft.begin() + ak,
                              std::min(input_spl_ft.begin() + 1 + bk, input_spl_ft.end()),
                              input_spl_ft.begin() + ak, 0.0);
    /*for (std::size_t i = ak; (i <= bk) && (i < data_spl.size()); i++) {
      sum += data_spl[i] * data_spl[i];
    }*/
    input_smoothed_spl_f_ft.push_back(f);
    input_smoothed_spl_ft.push_back(std::sqrt(m * sum));
  }

  std::vector<double> input_f_ft(n);
  std::generate(input_f_ft.begin(), input_f_ft.end(),
                [&, k = 0]() mutable { return k++ * 2 * static_cast<double>(sampleRate) / n; });

  auto graph = ui->plot->addGraph();
  graph->setData(QVector<double>{input_smoothed_spl_f_ft.begin(), input_smoothed_spl_f_ft.end()},
                 QVector<double>{input_smoothed_spl_ft.begin(), input_smoothed_spl_ft.end()}, true);
  graph->rescaleAxes();
  /*ui->plot->graph(1)->setData(QVector<double>::fromStdVector(data_f),
                              QVector<double>::fromStdVector(data_spl), true);*/
  ui->plot->graph(1)->rescaleAxes();
  ui->plot->xAxis->setRange(f0, ff);
  ui->plot->replot();

  // Reference signal's fft
  auto ref_fft = fft::r2c(mReferenceSignal);

  std::vector<double> mReferenceSignal_t(mReferenceSignal.size());
  std::iota(mReferenceSignal_t.begin(), mReferenceSignal_t.end(), 0);

  mQuickPlot.graph(0)->data().clear();
  mQuickPlot.graph(0)->addData(
      QVector<double>{mReferenceSignal_t.begin(), mReferenceSignal_t.end()},
      QVector<double>{mReferenceSignal.begin(), mReferenceSignal.end()});
  mQuickPlot.rescaleAxes();
  mQuickPlot.replot();
  mQuickPlot.show();

  std::vector<std::complex<double>> input_impulse_ft, input_impulse_2_ft;

  for (const auto& [input_c, ref_c] : ranges::zip_view(input_ft, ref_fft)) {
    input_impulse_ft.push_back(input_c / ref_c);
    input_impulse_2_ft.push_back(input_c);
  }
  auto input_impulse_ts   = fft::c2r(input_impulse_ft);
  auto input_impulse_2_ts = fft::c2r(input_impulse_2_ft);

  std::vector<double> input_impulse_t_ts(input_impulse_ts.size());
  std::generate(input_impulse_t_ts.begin(), input_impulse_t_ts.end(),
                [&, k = 0]() mutable { return k++ / static_cast<double>(sampleRate); });

  /*ui->plot2->clearGraphs();
  ui->plot2->addGraph();
  ui->plot2->graph(0)->addData(
      QVector<double>{input_impulse_t_ts.begin(), input_impulse_t_ts.end()},
      QVector<double>{input_impulse_ts.begin(), input_impulse_ts.end()});
  ui->plot2->graph(0)->rescaleAxes();
  ui->plot2->addGraph();
  ui->plot2->graph(1)->addData(
      QVector<double>{input_impulse_t_ts.begin(), input_impulse_t_ts.end()},
      QVector<double>{input_impulse_2_ts.begin(), input_impulse_2_ts.end()});
  ui->plot2->graph(1)->rescaleAxes();
  ui->plot2->graph(1)->setPen({Qt::red});
  ui->plot2->replot();*/

  // Restore ui
  ui->grpMeasParams->setEnabled(true);

  // Cleanup
  mBufferInput.close();
  mBufferOutput.close();
}

void MainWindow::on_bMeasure_clicked()
{
  // Get measurement parameters
  const auto f0         = ui->eStartFreq->value();
  const auto ff         = ui->eEndFreq->value();
  const auto length     = ui->eLength->currentData().toUInt();
  const auto sampleRate = ui->eSampleRate->currentData().toInt();
  const auto volume     = std::pow(10, ui->eVolumeDBFS->value() / 20.);
  const auto duration   = static_cast<double>(length) / sampleRate;

  // Compute factors
  const double k = std::pow(ff / f0, 1.0 / duration);

  // Configure output buffer & stream
  mBufferOutput.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Unbuffered);
  QDataStream out_stream(&mBufferOutput);
  out_stream.setByteOrder(QDataStream::LittleEndian);

  // 100m silence
  /*for (int i = 0; i < sampleRate / 10; i++) {
    out_stream << static_cast<int16_t>(0);
  }*/

  // Sine sweep sample data
  mReferenceSignal.clear();
  mReferenceSignal.reserve(length);
  for (uint32_t i = 0; i < length; i++) {
    const double t  = static_cast<double>(i) / sampleRate;
    const double kt = std::pow(k, t);
    const double amplitude =
        volume * std::sin(2 * std::numbers::pi * f0 * ((kt - 1) / std::log(k)));
    mReferenceSignal.push_back(amplitude);
    int16_t sample_amplitude = amplitude * mSampleFactor;
    out_stream << sample_amplitude;
  }

  // 100m silence
  for (int i = 0; i < sampleRate / 10; i++) {
    out_stream << static_cast<int16_t>(0);
  }

  mBufferOutput.close();

  // Setup format
  QAudioFormat format;
  format.setSampleRate(sampleRate);
  format.setChannelCount(1);
  format.setSampleSize(mSampleSize);
  format.setCodec("audio/pcm");
  format.setByteOrder(QAudioFormat::LittleEndian);
  format.setSampleType(QAudioFormat::SignedInt);

  QAudioDeviceInfo outputInfo(QAudioDeviceInfo::defaultOutputDevice());
  QAudioDeviceInfo inputInfo(QAudioDeviceInfo::defaultInputDevice());
  if (!outputInfo.isFormatSupported(format)) {
    qWarning() << "Raw audio format not supported by backend, cannot play audio.";
    return;
  }
  if (!inputInfo.isFormatSupported(format)) {
    qWarning() << "Raw audio format not supported by backend, cannot record audio.";
    return;
  }

  // Disable further editing
  ui->grpMeasParams->setEnabled(false);

  // Start output & input
  mAudioOutput = std::make_unique<QAudioOutput>(format, this);
  mAudioInput  = std::make_unique<QAudioInput>(format, this);

  mAudioInput->setBufferSize(0);
  mAudioOutput->setBufferSize(0);

  qDebug() << "bs:" << mAudioOutput->bufferSize();
  qDebug() << "bs:" << mAudioInput->bufferSize();

  mBufferOutput.open(QIODevice::ReadOnly | QIODevice::Unbuffered);
  mBufferInput.open(QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Unbuffered);

  const int notifyInterval = 1000 * (mBufferOutput.size() / mBytesPerSample) / sampleRate;
  mAudioInput->setNotifyInterval(notifyInterval);

  // connect(mAudioInput.get(), &QAudioInput::notify, this, &MainWindow::handleFinished);
  connect(mAudioOutput.get(), &QAudioOutput::stateChanged, [&](const QAudio::State& state) {
    if (state == QAudio::IdleState)
      handleFinished();
  });

  mAudioOutput->start(&mBufferOutput);
  mAudioInput->start(&mBufferInput);
}

void MainWindow::updateMeasurementDuration()
{
  const auto sampleRate = ui->eSampleRate->currentData().toInt();
  const auto length     = ui->eLength->currentData().toUInt();

  // Duration [s] = Length [samples] / SampleRate [samples/s]
  ui->lvDuration->setText(QString::number(static_cast<double>(length) / sampleRate, 'f', 1) + " s");
}

void MainWindow::handleRecordedData()
{
  QDataStream input_stream(mBufferInput.data());
  input_stream.setByteOrder(QDataStream::LittleEndian);

  const auto sampleRate = ui->eSampleRate->currentData().toInt();

  ui->plot2->graph(0)->data()->clear();
  int i = 0;
  while (!input_stream.atEnd()) {
    double  t = static_cast<double>(i++) / sampleRate;
    int16_t sample_amplitude;
    input_stream >> sample_amplitude;
    ui->plot2->graph(0)->addData(t, sample_amplitude);
  }

  ui->plot2->graph(0)->rescaleAxes();
  ui->plot2->replot();
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

#include "mainwindow.hh"
#include "./ui_mainwindow.h"

#include <algorithm>
#include <numbers>

#include <QDebug>

#include <fft.hh>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  setupUi();
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::setupUi()
{
  ui->plot->addGraph();
  ui->plot->addGraph();
  ui->plot->graph(1)->setPen(QPen{Qt::red});
  ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

  // Supported sample rates combo box
  const auto outputSR = QAudioDeviceInfo::defaultOutputDevice().supportedSampleRates();
  const auto inputSR  = QAudioDeviceInfo::defaultInputDevice().supportedSampleRates();
  const auto supportedSRset = QSet<int>{outputSR.begin(), outputSR.end()}.intersect(
      {inputSR.begin(), inputSR.end()});
  auto supportedSR = QList<int>{supportedSRset.begin(), supportedSRset.end()};
  std::sort(supportedSR.begin(), supportedSR.end());

  for (auto sampleRate : supportedSR) {
    ui->eSampleRate->addItem(QString::number(sampleRate), sampleRate);
  }
  /*auto selectedSR = std::find_if(supportedSR.begin(), supportedSR.end(),
                                 [](const auto sr) { return sr >= 44100; });
  ui->eSampleRate->setCurrentIndex(std::distance(supportedSR.begin(), selectedSR));*/
  ui->eSampleRate->setCurrentText("192000");

  // Length
  ui->eLength->addItem("128k", 128000u);
  ui->eLength->addItem("256k", 256000u);
  ui->eLength->addItem("512k", 512000u);
  ui->eLength->addItem("1M", 1024000u);

  // Connections
  connect(ui->eSampleRate, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateMeasurementDuration);
  connect(ui->eLength, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &MainWindow::updateMeasurementDuration);

  // Update ui
  updateMeasurementDuration();
}

void MainWindow::handleFinished(QAudio::State state)
{
  if (state == QAudio::State::IdleState) {
    // Stop measurement
    mAudioOutput->stop();
    mAudioInput->stop();

    // Get measurement parameters
    const double f0         = ui->eStartFreq->value();
    const double ff         = ui->eEndFreq->value();
    const auto   length     = ui->eLength->currentData().toUInt();
    const auto   sampleRate = ui->eSampleRate->currentData().toInt();
    const double duration   = static_cast<double>(length) / sampleRate;

    // Compute factors
    const double sample_amplitude_factor = 2. / ((1 << mSampleSize) - 1);

    // Configure input buffer & stream
    mBufferInput.close();
    mBufferInput.open(QIODevice::ReadOnly);
    QDataStream input(&mBufferInput);
    input.setByteOrder(QDataStream::LittleEndian);

    uint32_t            i = 0;
    std::vector<double> times, amplitudes;
    while (!input.atEnd()) {
      double  t = static_cast<double>(i) / sampleRate;
      int16_t sample_amplitude;
      input >> sample_amplitude;
      double amplitude = sample_amplitude * sample_amplitude_factor;

      times.push_back(t);
      amplitudes.push_back(amplitude);
      i++;
    }
    qDebug() << "Samples received:" << i;
    const auto n = i;

    auto amplitudes_fft = fft::r2c(amplitudes);

    qDebug() << "FFT size:" << amplitudes_fft.size();

    constexpr double sensitivitydB = 4.6889;

    std::vector<double> data_f, data_spl;

    i = 0;
    for (auto d : amplitudes_fft) {
      double f = i * static_cast<double>(sampleRate) / n;
      i++;
      if (f < f0 || f > ff)
        continue;
      const double normalized_magnitude = 2 * std::abs(d) / n;
      const double dbfs                 = 20 * std::log10(normalized_magnitude);
      const double spl                  = 120 + dbfs - sensitivitydB;

      data_f.push_back(f);
      data_spl.push_back(spl);
    }

    int octave = 6;

    std::vector<double> smoothed_spl;
    for (std::size_t k = 0; k < data_spl.size(); k++) {
      const double f = data_f[k];
      if (f <= 100)
        octave = 48;
      else if (f <= 1000)
        octave = 48 - (f - 100) / 900 * 42;  // from 48 to 6 at 1000 Hz
      else if (f >= 10000)
        octave = 3;
      const double      a_fac = std::pow(2., -1. / (2. * octave));
      const double      b_fac = std::pow(2., 1. / (2. * octave));
      const std::size_t ak    = std::round(k * a_fac);
      const std::size_t bk    = std::round(k * b_fac);
      const std::size_t a     = std::round(f * a_fac);
      const std::size_t b     = std::round(f * b_fac);
      const double      m     = 1. / (bk - ak + 1);
      double            sum   = 0;
      for (std::size_t i = ak; (i <= bk) && (i < data_spl.size()); i++) {
        sum += data_spl[i] * data_spl[i];
      }
      smoothed_spl.push_back(std::sqrt(m * sum));
    }

    ui->plot->graph(0)->setData(QVector<double>::fromStdVector(data_f),
                                QVector<double>::fromStdVector(smoothed_spl));
    ui->plot->graph(1)->setData(QVector<double>::fromStdVector(data_f),
                                QVector<double>::fromStdVector(data_spl));
    ui->plot->graph(0)->rescaleAxes();
    ui->plot->graph(1)->rescaleAxes();
    ui->plot->replot();

    // Restore ui
    ui->grpMeasParams->setEnabled(true);

    // Cleanup
    mBufferInput.close();
    mBufferOutput.close();
  }
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
  const double sample_amplitude_factor = ((1 << mSampleSize) - 1) / 2.;
  const double k                       = std::pow(ff / f0, 1.0 / duration);

  // Configure output buffer & stream
  mBufferOutput.open(QIODevice::WriteOnly | QIODevice::Truncate);
  QDataStream out_stream(&mBufferOutput);
  out_stream.setByteOrder(QDataStream::LittleEndian);

  // Sine sweep sample data
  for (uint32_t i = 0; i < length; i++) {
    const double t  = static_cast<double>(i) / sampleRate;
    const double kt = std::pow(k, t);
    const double amplitude =
        volume * std::sin(2 * std::numbers::pi * f0 * ((kt - 1) / std::log(k)));
    int16_t sample_amplitude = amplitude * sample_amplitude_factor;
    out_stream << sample_amplitude;
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

  mAudioOutput = std::make_unique<QAudioOutput>(format, this);
  mAudioInput  = std::make_unique<QAudioInput>(format, this);

  mBufferOutput.open(QIODevice::ReadOnly);
  mBufferInput.open(QIODevice::WriteOnly | QIODevice::Truncate);

  mAudioOutput->start(&mBufferOutput);
  mAudioInput->start(&mBufferInput);

  connect(mAudioOutput.get(), &QAudioOutput::stateChanged, this,
          &MainWindow::handleFinished);
}

void MainWindow::updateMeasurementDuration()
{
  auto sampleRate = ui->eSampleRate->currentData().toInt();
  auto length     = ui->eLength->currentData().toUInt();

  // Duration [s] = Length [samples] / SampleRate [samples/s]
  ui->lvDuration->setText(
      QString::number(static_cast<double>(length) / sampleRate, 'f', 1) + " s");
}

void MainWindow::on_eStartFreq_valueChanged(int f0)
{
  auto ff = ui->eEndFreq->value();
  if (ff <= f0) {
    ui->eEndFreq->setValue(f0 + 1);
  }
}

void MainWindow::on_eEndFreq_valueChanged(int ff)
{
  auto f0 = ui->eStartFreq->value();
  if (f0 >= ff) {
    ui->eStartFreq->setValue(ff - 1);
  }
}

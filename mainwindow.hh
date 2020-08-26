#pragma once

#include <QAudioInput>
#include <QAudioOutput>
#include <QBuffer>
#include <QMainWindow>

#include <qcustomplot/qcustomplot.h>

#include <memory>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 private:
  static constexpr uint32_t mSampleSize     = 16;
  static constexpr uint32_t mBytesPerSample = mSampleSize / 8;
  static constexpr double   mSampleFactor   = ((1 << mSampleSize) - 1) / 2.;

 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  void setupUi();

 private slots:
  void on_eStartFreq_valueChanged(int f0);
  void on_eEndFreq_valueChanged(int ff);

  void on_bMeasure_clicked();

 private:
  void updateMeasurementDuration();

  void handleRecordedData();
  void handleFinished();

 private:
  Ui::MainWindow* ui;

  std::unique_ptr<QAudioOutput> mAudioOutput;
  std::unique_ptr<QAudioInput>  mAudioInput;
  QBuffer                       mBufferOutput, mBufferInput;
  std::vector<double>           mReferenceSignal;
  QCustomPlot                   mQuickPlot;
};

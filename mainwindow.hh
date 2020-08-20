#pragma once

#include <QAudioInput>
#include <QAudioOutput>
#include <QBuffer>
#include <QMainWindow>

#include <qcustomplot/qcustomplot.h>

#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 private:
  static constexpr uint32_t mSampleSize = 16;

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

  void handleFinished(QAudio::State state);

 private:
  Ui::MainWindow* ui;

  std::unique_ptr<QAudioOutput> mAudioOutput;
  std::unique_ptr<QAudioInput>  mAudioInput;
  QBuffer                       mBufferOutput, mBufferInput;
};

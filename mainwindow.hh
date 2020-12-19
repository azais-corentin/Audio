#pragma once

#include "audioio.hh"

#include <qcustomplot/qcustomplot.h>
#include <QMainWindow>

#include <memory>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE
class QtAwesome;

namespace Audio {

class MainWindow : public QMainWindow {
  Q_OBJECT

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

  void handleFinished();

 private:
  Ui::MainWindow* ui;
  QtAwesome*      mAwesome;

  AudioIO     mAudio;
  QCustomPlot mQuickPlot;
};

}  // namespace Audio

#pragma once

#include "audioio.hh"

#include <QMainWindow>
#include <qcustomplot/qcustomplot.h>

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
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setupUi();

    void quickPlot(std::string name, const std::vector<float> &signal, float scaling = 1);

  private slots:
    void on_eStartFreq_valueChanged(int f0);
    void on_eEndFreq_valueChanged(int ff);
    void on_bMeasure_clicked();

    void on_bToggleSpectrogram_clicked();

  private:
    void updateMeasurementDuration();

    void handle_finished();
    void handle_rta_data();

  private:
    Ui::MainWindow *ui;
    QtAwesome *awesome_;

    AudioIO audio_;
    QCustomPlot quick_plot_;
    QCPColorMap *color_map_;
};

} // namespace Audio

#pragma once

#include "audioio.hh"

#include <QMainWindow>
#include <qcustomplot.h>

#include <memory>
#include <vector>

QT_BEGIN_NAMESPACE

namespace Ui
{
class MainWindow;
} // namespace Ui

QT_END_NAMESPACE

// NOLINTNEXTLINE(readability-identifier-naming)
namespace fa
{
class QtAwesome;
} // namespace fa

namespace Audio
{

class MainWindow final : public QMainWindow
{
    Q_OBJECT // NOLINT

  public:
    explicit MainWindow(QWidget *parent = nullptr);
    MainWindow(const MainWindow &)            = delete;
    MainWindow(MainWindow &&)                 = delete;
    MainWindow &operator=(const MainWindow &) = delete;
    MainWindow &operator=(MainWindow &&)      = delete;
    ~MainWindow() final;

    // void quickPlot(std::string name, const std::vector<float> &signal, float scaling = 1);

  private slots:
    void updateStartFrequency(int frequencyInitial);
    void updateEndFrequency(int frequencyFinal);
    void startMeasurement();
    // void toggleRTA();

  private:
    void setupUi();
    void updateMeasurementDuration();

    void handleFinished();
    // void handleRtaData();

    void setupPlotForLogLine(QCustomPlot &plot);

  private:
    Ui::MainWindow *ui_;
    std::unique_ptr<fa::QtAwesome> awesome_;

    AudioIO audio_;
    // QCustomPlot quickPlot_;

    std::unique_ptr<QCPColorMap> colorMap_;
    std::unique_ptr<QCPColorScale> colorScale_;
    std::unique_ptr<QCPMarginGroup> marginGroup_;
};

} // namespace Audio

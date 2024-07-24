#include "mainwindow.hh"

#include <QApplication>
#include <QFont>
#include <QSplashScreen>
#include <spdlog/spdlog.h>

int main(int argc, char *argv[])
{
    qputenv("QT_QPA_PLATFORM", "windows:darkmode=0");

    QApplication application(argc, argv);
    application.setStyle("fusion");
    spdlog::info("Current style: {}", application.style()->name().toStdString());

    // Splash screen
    QPixmap pixmap = QPixmap{"assets/splash.png"}.scaledToHeight(200, Qt::SmoothTransformation);
    QSplashScreen splash(pixmap);
    QFont calibri{"Calibri", 14, 300};
    calibri.setLetterSpacing(QFont::PercentageSpacing, 130);
    calibri.setCapitalization(QFont::AllUppercase);
    splash.setFont(calibri);
    splash.showMessage("\nLoading", Qt::AlignCenter, Qt::white);
    splash.show();

    application.processEvents();

    Audio::MainWindow window;
    window.show();
    splash.finish(&window);
    return application.exec();
}

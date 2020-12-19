#include "mainwindow.hh"

#include <QApplication>

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);

  QPixmap       pixmap("splash.png");
  QSplashScreen splash(pixmap);
  splash.showMessage("Loading application...", Qt::AlignCenter, QColor(78, 0, 22));
  splash.show();
  a.processEvents();

  Audio::MainWindow w;
  w.show();
  splash.finish(&w);
  return a.exec();
}

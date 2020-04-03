#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication a(argc, argv);
  a.setWindowIcon(QIcon(":/icon/res/icon/typora.png"));

  MainWindow w;
  w.showMaximized();

  QApplication::setQuitOnLastWindowClosed(false);

  return a.exec();
}




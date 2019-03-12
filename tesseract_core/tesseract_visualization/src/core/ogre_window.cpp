#include "tesseract_visualization/core/qt_ogre_window.h"
#include <QApplication>

int main(int argc, char** argv)
{
  using namespace tesseract_visualization;
  QApplication app(argc, argv);

  QTOgreWindow* ogreWindow = new QTOgreWindow();
  ogreWindow->show();

  return app.exec();
}

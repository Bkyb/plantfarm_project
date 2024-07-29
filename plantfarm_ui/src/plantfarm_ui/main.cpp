#include <QApplication>
#include <QIcon>
#include "plantfarm_ui/plantfarm_ui.hpp"


int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  plantfarm_ui w;

  // set the window title as the node name
  w.setWindowTitle(QString::fromStdString("Plantfarm UI"));

  // load the icon from our qrc file and set it as the application icon
  //QIcon icon(":/icons/my_gui_icon.png");
  //w.setWindowIcon(icon);

  w.show();
  return a.exec();
}
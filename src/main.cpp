/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/cam_udp_publisher/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  cam_udp_publisher::MainWindow w(argc, argv);
  w.setFixedSize(1024, 600);
  // w.setMenuBar(nullptr);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}

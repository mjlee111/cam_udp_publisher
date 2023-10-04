/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/cam_udp_publisher/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cam_udp_publisher
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/camera.png"));
  QPixmap pixmap(":/images/intelligence.png");
  ui.logo->setPixmap(pixmap.scaled(ui.logo->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

  qnode.init();

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

}  // namespace cam_udp_publisher

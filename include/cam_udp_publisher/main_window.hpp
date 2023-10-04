/**
 * @file /include/cam_udp_publisher/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef cam_udp_publisher_MAIN_WINDOW_H
#define cam_udp_publisher_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "udp.h"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace cam_udp_publisher
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

public Q_SLOTS:

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace cam_udp_publisher

#endif  // cam_udp_publisher_MAIN_WINDOW_H

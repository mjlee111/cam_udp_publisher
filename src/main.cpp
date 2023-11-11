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

#include "../include/cam_udp_publisher/main_window.hpp"
#include <QApplication>
#include <QtGui>

/*****************************************************************************
** Main
*****************************************************************************/

int main (int argc, char** argv) {
    /*********************
    ** Qt
    **********************/
    QApplication app (argc, argv);
    cam_udp_publisher::MainWindow w (argc, argv);

    w.show ();
    app.connect (&app, SIGNAL (lastWindowClosed ()), &app, SLOT (quit ()));
    int result = app.exec ();

    return result;
}

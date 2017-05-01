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
#include "../include/interface/interface/main_window.hpp"
#include "roboy_managing_node/myoMaster.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    MyoMaster myoMaster(argc,argv);
    myoMaster.initialize();

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    interface::MainWindow w(argc,argv);
    w.myoMaster = &myoMaster;
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}

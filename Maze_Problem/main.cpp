#include "mainwindow.h"

#include <QApplication>
#include <QSurfaceFormat>

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);


    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(3,3);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);//固定功能管线

    MainWindow w;
    w.show();
    return a.exec();
}

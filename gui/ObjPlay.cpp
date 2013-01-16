#include <QtGui/QApplication>
#include "ObjLoadThread.h"
#include "ObjMainWindow.h"

int
main(int argc, char* argv[])
{
    ObjLoadThread& thread(ObjLoadThread::instance());

    QApplication a(argc, argv);
    ObjMainWindow w;
    w.connect(&thread, SIGNAL(frameLoaded()), SLOT(frameLoaded()));
    w.raise();
    w.show();

    thread.init("frames");
    thread.start(QThread::NormalPriority);

    return a.exec();
}

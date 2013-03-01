#include <QtGui/QApplication>
#include <gui/playback/ObjLoadThread.h>
#include <gui/playback/ObjMainWindow.h>

int
main(int argc, char* argv[])
{
    ObjLoadThread& thread(ObjLoadThread::instance());

    QApplication a(argc, argv);
    ObjMainWindow w;
    w.connect(&thread, SIGNAL(frameLoaded()), SLOT(frameLoaded()));
    w.raise();
    w.show();

    std::string dir = argc > 1 ? argv[1] : "frames";
    thread.init(dir);
    thread.start(QThread::NormalPriority);

    return a.exec();
}

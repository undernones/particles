#include <QtGui/QApplication>
#include <physics/SoftBody.h>
#include "MainWindow.h"
#include "SimThread.h"
#include "../Options.h" // TODO: dependencies!
#include "../World.h" // TODO: dependencies!

int main(int argc, char* argv[])
{
    Options::init(argc, argv);

    Material material = {
        Options::mu(),
        Options::lambda(),
        Options::density(),
    };
    SoftBody body(Options::particleFile(), material);
    World::addSoftBody(&body);

    SimThread& thread(SimThread::instance());
    // TODO: Fix the backwards logic. Something is wrong with the
    // options parsing.
    if (!Options::startPaused()) {
        thread.pause();
    }

    QApplication a(argc, argv);
    MainWindow w;
    w.setSoftBody(World::bodies()[0]);
    w.connect(&thread, SIGNAL(stepped()), SLOT(stepped()));
    w.raise();
    w.show();

    thread.init(Options::duration(), Options::dt());
    thread.start(QThread::NormalPriority);

    return a.exec();
}
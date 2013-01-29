#include <QtGui/QApplication>
#include <geom/Mesh.h>
#include <physics/SoftBody.h>
#include <simulator/FrameSaver.h>
#include <simulator/SimThread.h>
#include <simulator/World.h>
#include "SimMainWindow.h"
#include "../Options.h" // TODO: dependencies!

int
main(int argc, char* argv[])
{
    Options::init(argc, argv);
    World::init();

    SimThread& thread(SimThread::instance());
    // TODO: Fix the backwards logic. Something is wrong with the
    // options parsing.
    if (!Options::startPaused()) {
        thread.pause();
    }

    const SoftBody* body = World::bodies()[0];

    QApplication a(argc, argv);
    SimMainWindow w;
    w.setSoftBody(body);
    w.connect(&thread, SIGNAL(stepped()), SLOT(stepped()));
    w.raise();
    w.show();

    FrameSaver saver(*body, Options::framesDir(), Options::dt(), Options::fps());
    if (body->mesh() != nullptr) {
        saver.connect(&thread, SIGNAL(stepped()), SLOT(stepped()));
    }

    thread.init(Options::duration(), Options::dt());
    thread.start(QThread::NormalPriority);

    return a.exec();
}

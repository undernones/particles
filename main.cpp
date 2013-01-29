#include <QtGui/QApplication>
#include <iostream>
#include <stdexcept>
#include <physics/SoftBody.h>
#include <simulator/FrameSaver.h>
#include <simulator/SimThread.h>
#include <simulator/World.h>
#include "Options.h"
#include "Utils.h"

int
main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    Options::init(argc, argv);
    World::init();

    SimThread& thread(SimThread::instance());

    const SoftBody* body = World::bodies()[0];
    FrameSaver saver(*body, Options::framesDir(), Options::dt(), Options::fps());
    if (body->mesh() != nullptr) {
        saver.connect(&thread, SIGNAL(stepped()), SLOT(stepped()));
    }

    uint32_t totalSteps = Options::duration() / Options::dt();
    std::cout << "Duration:    " << Options::duration() << "s" << std::endl
              << "Timestep:    " << Options::dt() << "s" << std::endl
              << "Total steps: " << totalSteps << std::endl
              ;

    thread.init(Options::duration(), Options::dt());
    thread.start(QThread::NormalPriority);

    return a.exec();
}

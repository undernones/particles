#include <QtCore/QtCore>
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
    Eigen::initParallel();

    QCoreApplication a(argc, argv);
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
              << std::endl
              << "mu:          " << Options::mu() << std::endl
              << "lambda:      " << Options::lambda() << std::endl
              << "flow rate:   " << Options::flowRate() << std::endl
              << "yield point: " << Options::yieldPoint() << std::endl
              << "hardening:   " << Options::hardening() << std::endl
              ;

    a.connect(&thread, SIGNAL(finished()), SLOT(quit()));

    thread.init(Options::duration(), Options::dt());
    thread.start(QThread::NormalPriority);

    return a.exec();
}

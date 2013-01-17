#include <QtGui/QApplication>
#include <geom/Mesh.h>
#include <physics/PlaneObstacle.h>
#include <physics/SoftBody.h>
#include "FrameSaver.h"
#include "SimMainWindow.h"
#include "SimThread.h"
#include "../Options.h" // TODO: dependencies!
#include "../World.h" // TODO: dependencies!

int
main(int argc, char* argv[])
{
    Options::init(argc, argv);

    Material material = {
        Options::mu(),
        Options::lambda(),
        Options::density(),
    };
    SoftBody body(Options::particleFile(), material);
    Mesh mesh;
    if (Mesh::loadObj(Options::meshFile(), mesh)) {
        body.setMesh(&mesh);
    }

    World::addSoftBody(&body);
    World::addObstacle(new PlaneObstacle(Eigen::Vector3d(0, 1, 0), -1, Options::friction()));

    SimThread& thread(SimThread::instance());
    // TODO: Fix the backwards logic. Something is wrong with the
    // options parsing.
    if (!Options::startPaused()) {
        thread.pause();
    }

    QApplication a(argc, argv);
    SimMainWindow w;
    w.setSoftBody(World::bodies()[0]);
    w.connect(&thread, SIGNAL(stepped()), SLOT(stepped()));
    w.raise();
    w.show();

    FrameSaver saver(body, Options::framesDir(), Options::dt(), Options::fps());
    if (body.mesh() != nullptr) {
        saver.connect(&thread, SIGNAL(stepped()), SLOT(stepped()));
    }

    thread.init(Options::duration(), Options::dt());
    thread.start(QThread::NormalPriority);

    return a.exec();
}

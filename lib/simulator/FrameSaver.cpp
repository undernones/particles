#include "FrameSaver.h"
#include <geom/Mesh.h>
#include <physics/SoftBody.h>
#include "Options.h"
#include "World.h"

FrameSaver::FrameSaver() : QObject()
{
    mElapsed = std::numeric_limits<double>::max();
    mSpf = 1.0 / Options::fps();
    mFrame = 0;
}

FrameSaver::~FrameSaver()
{
}

void
FrameSaver::stepped()
{
    if (mElapsed > mSpf) {
        for (auto body : World::bodies()) {
            // TODO: If there is more than one body, they will overwrite each
            // other!
            body->mesh()->saveObj(Mesh::makeObjName(Options::framesDir(), mFrame));
        }
        mElapsed = 0;
        emit savedFrame(mFrame);
        mFrame++;
    }

    mElapsed += Options::dt();
}

#include "FrameSaver.h"
#include <fstream>
#include <geom/Mesh.h>
#include <physics/SoftBody.h>
#include "Options.h"
#include "World.h"

namespace
{

std::string
makeParticleFilename(const std::string& dir, uint32_t frameNum)
{
    static char result[1024];
    std::string format(dir + "/world.%06u");
    sprintf(result, format.c_str(), frameNum);
    return std::string(result);
}

bool
dumpWorldParticles(const std::string& filename, const SoftBody& body)
{
    std::ofstream outf(filename.c_str(), std::ios::out);
    if (!outf.is_open()) return false;

    for (auto& x : body.posWorld) {
        outf << x.transpose() << std::endl;
    }
    return true;
}

}

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
            assert(body != nullptr);

            // TODO: If there is more than one body, they will overwrite each
            // other!
            std::string filename = makeParticleFilename("/tmp", mFrame);
            dumpWorldParticles(filename, *body);
            body->mesh()->saveObj(Mesh::makeObjName(Options::framesDir(), mFrame));
        }
        mElapsed = 0;
        emit savedFrame(mFrame);
        mFrame++;
    }

    mElapsed += Options::dt();
}

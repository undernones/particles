#include "FrameSaver.h"
#include <geom/Mesh.h>
#include <physics/SoftBody.h>

namespace
{

std::string
makeObjName(const std::string& dir, uint32_t frameNum)
{
    static char result[1024];
    std::string format(dir + "/surface.%06u.obj");
    sprintf(result, format.c_str(), frameNum);
    return std::string(result);
}

}

FrameSaver::FrameSaver(const SoftBody& body, const std::string& framesDir, double dt, double fps)
:   QObject()
,   mBody(body)
,   mFramesDir(framesDir)
,   mDt(dt)
,   mFps(fps)
,   mElapsed(std::numeric_limits<double>::max())
,   mSpf(1.0 / fps)
,   mFrame(0)
{
}

FrameSaver::~FrameSaver()
{
}

void
FrameSaver::stepped()
{
    if (mElapsed > mSpf) {
        mBody.mesh()->saveObj(makeObjName(mFramesDir, mFrame));
        mElapsed = 0;
        mFrame++;
    }

    mElapsed += mDt;
}

#include "FrameSaver.h"
#include <geom/Mesh.h>
#include <physics/SoftBody.h>

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
        mBody.mesh()->saveObj(Mesh::makeObjName(mFramesDir, mFrame));
        mElapsed = 0;
        mFrame++;
    }

    mElapsed += mDt;
}

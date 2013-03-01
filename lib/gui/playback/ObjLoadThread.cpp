#include "ObjLoadThread.h"
#include "MeshSet.h"

ObjLoadThread::ObjLoadThread(QObject* parent)
:   QThread(parent)
,   mQuitFlag(false)
,   mDir()
{
}

ObjLoadThread::~ObjLoadThread()
{
}

ObjLoadThread&
ObjLoadThread::instance()
{
    static ObjLoadThread inst;
    return inst;
}

void
ObjLoadThread::run()
{
    uint32_t i = 0;
    uint32_t sleepMs = 0;

    while (!mQuitFlag) {
        Mesh* mesh = new Mesh();
        if (Mesh::loadObj(Mesh::makeObjName(mDir, i), *mesh)) {
            MeshSet::instance().push_back(mesh);
            sleepMs = 0;
            emit frameLoaded();
            i++;
        } else {
            sleepMs = std::max(sleepMs + 10, (uint32_t)1000);
        }
        this->msleep(sleepMs);
    }
}

void
ObjLoadThread::quit()
{
    mQuitFlag = true;
    QThread::quit();
}

void
ObjLoadThread::init(const std::string& dir)
{
    if (isRunning()) return;
    mDir = dir;
}

#include "MeshSet.h"

MeshSet::MeshSet() : mCurrent(std::numeric_limits<uint32_t>::max())
{
    reserve(2000); // Enough for over a minute
}

MeshSet::~MeshSet()
{
    for (auto m : *this) {
        delete m;
    }
}

MeshSet&
MeshSet::instance()
{
    static MeshSet result;
    return result;
}

const Mesh*
MeshSet::current() const
{
    return mCurrent < size() ? (*this)[mCurrent] : nullptr;
}

void
MeshSet::rewind()
{
    goTo(0);
}

bool
MeshSet::goTo(uint32_t frame)
{
    if (frame < size()) {
        mCurrent = frame;
        return true;
    } else {
        mCurrent = size() - 1;
        return false;
    }
}

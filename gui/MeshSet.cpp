#include "MeshSet.h"

MeshSet::MeshSet()
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

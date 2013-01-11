#include "Neighborhood.h"

const uint32_t Neighborhood::MAX_SIZE = 8;

Neighborhood::Neighborhood() : Collection<Neighbor>()
,   mSum(0)
{
    reserve(MAX_SIZE);
}

bool
Neighborhood::hasNeighbor(uint32_t j)
{
    for (auto& n : *this) {
        if (n.index == j) return true;
    }
    return false;
}

Neighbor
Neighborhood::findNeighbor(uint32_t j)
{
    for (auto& n : *this) {
        if (n.index == j) return n;
    }
    return Neighbor();
}

double
Neighborhood::computeSum()
{
    mSum = 0;
    for (auto& n : *this) {
        mSum += n.w;
    }
    return mSum;
}

double
Neighborhood::sum() const
{
    return mSum;
}

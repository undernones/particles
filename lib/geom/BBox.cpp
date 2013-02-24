#include "BBox.h"
#include <float.h>

BBox::BBox() :
    lo( DBL_MAX,  DBL_MAX,  DBL_MAX),
    hi(-DBL_MAX, -DBL_MAX, -DBL_MAX)
{
}

bool
BBox::contains(const Eigen::Vector3d& p) const
{
    return lo[0] <= p[0] && p[0] <= hi[0]
        && lo[1] <= p[1] && p[1] <= hi[1]
        && lo[2] <= p[2] && p[2] <= hi[2]
    ;
}

void
BBox::add(const BBox& other)
{
    if (other.lo[0] < lo[0]) lo[0] = other.lo[0];
    if (other.lo[1] < lo[1]) lo[1] = other.lo[1];
    if (other.lo[2] < lo[2]) lo[2] = other.lo[2];

    if (other.hi[0] > hi[0]) hi[0] = other.hi[0];
    if (other.hi[1] > hi[1]) hi[1] = other.hi[1];
    if (other.hi[2] > hi[2]) hi[2] = other.hi[2];
}

void
BBox::add(const Eigen::Vector3d& p)
{
    if (p[0] < lo[0]) lo[0] = p[0];
    if (p[1] < lo[1]) lo[1] = p[1];
    if (p[2] < lo[2]) lo[2] = p[2];

    if (p[0] > hi[0]) hi[0] = p[0];
    if (p[1] > hi[1]) hi[1] = p[1];
    if (p[2] > hi[2]) hi[2] = p[2];
}

void
BBox::add(const VectorList& points)
{
    for (auto& p : points) {
        add(p);
    }
}

Eigen::Vector3d
BBox::center() const
{
    Eigen::Vector3d toCenter = 0.5 * (hi - lo);
    return lo + toCenter;
}

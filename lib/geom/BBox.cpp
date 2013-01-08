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

BBox
BBox::unionWith(const BBox& other) const
{
    BBox result(*this);

    if (other.lo[0] < lo[0]) result.lo[0] = other.lo[0];
    if (other.lo[1] < lo[1]) result.lo[1] = other.lo[1];
    if (other.lo[2] < lo[2]) result.lo[2] = other.lo[2];

    if (other.hi[0] > hi[0]) result.hi[0] = other.hi[0];
    if (other.hi[1] > hi[1]) result.hi[1] = other.hi[1];
    if (other.hi[2] > hi[2]) result.hi[2] = other.hi[2];

    return result;
}


#include "PlaneObstacle.h"

PlaneObstacle::PlaneObstacle() :
    mNormal()
{
    mOffset = 0;
}

PlaneObstacle::PlaneObstacle(const Eigen::Vector3d& normal, double offset, double friction) :
    Obstacle(friction),
    mNormal(normal),
    mOffset(offset)
{
    mNormal.normalize();
}

PlaneObstacle::~PlaneObstacle()
{
}

bool
PlaneObstacle::bounce(Eigen::Vector3d& x, Eigen::Vector3d& v) const
{
    double d = x.dot(mNormal);
    if (d > mOffset) return false;

    // Project the position onto the surface of the obstacle.
    x += (mOffset - d) * mNormal;

    // Split the velocity into normal and tangential components.
    double normMag = v.dot(mNormal);
    Eigen::Vector3d normVel = normMag * mNormal;
    Eigen::Vector3d tanVel = v - normVel;

    v = (1.0 - friction()) * tanVel;
    return true;
}

void
PlaneObstacle::draw() const
{
}


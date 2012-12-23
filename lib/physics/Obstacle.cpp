#include "Obstacle.h"

Obstacle::Obstacle() :
    mFriction(0.05)
{
}

Obstacle::Obstacle(double friction) :
    mFriction(friction)
{
}

Obstacle::~Obstacle()
{
}

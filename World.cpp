#include "World.h"
#include <physics/SoftBody.h>

std::vector<SoftBody*> World::sBodies;

void
World::addSoftBody(SoftBody* body)
{
    sBodies.push_back(body);
}

void
World::step(double dt)
{
    for (auto& body : sBodies) {
        body->clearForces();
        body->computeInternalForces();
    }
}

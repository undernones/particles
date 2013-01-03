#include "World.h"
#include <physics/SoftBody.h>
#include "Options.h"

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

        // Integrate
        auto x_it = body->worldPositions().begin();
        auto v_it = body->velocities().begin();
        auto f_it = body->forces().begin();
        auto m_it = body->masses().begin();
        for (; f_it != body->forces().end(); ++x_it, ++v_it, ++f_it, ++m_it) {
            static Eigen::Vector3d g(0, Options::gravity(), 0);
            auto a = *f_it / *m_it + g;
            *v_it += dt * a;
            *x_it += dt * *v_it;
        }

        // Collide
        //for (auto o : sObstacles) {
        //    for (auto& vert : sBody.mesh().verts) {
        //        if (o->bounce(vert)) {
        //            //std::cout << "hit!" << std::endl;
        //        }
        //    }
        //}
    }
}

const std::vector<const SoftBody*>
World::bodies()
{
    return std::vector<const SoftBody*>(sBodies.begin(), sBodies.end());
}

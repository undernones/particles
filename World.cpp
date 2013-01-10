#include "World.h"
#include <physics/Obstacle.h>
#include <physics/SoftBody.h>
#include "Options.h"

std::vector<SoftBody*> World::sBodies;
std::vector<Obstacle*> World::sObstacles;

namespace
{

void
integrateExplicit(double dt, SoftBody& body)
{
    auto x_it = body.posWorld.begin();
    auto v_it = body.velocities.begin();
    auto f_it = body.forces.begin();
    auto m_it = body.masses.begin();
    for (; f_it != body.forces.end(); ++x_it, ++v_it, ++f_it, ++m_it) {
        static Eigen::Vector3d g(0, Options::gravity(), 0);
        auto a = *f_it / *m_it + g;
        *v_it += dt * a;
        *x_it += dt * *v_it;
    }
}

//void
//integrateLeapfrog(double dt, SoftBody& body)
//{
//    // TODO: Debug this. It doesn't seem to work.
//
//    double dt_2 = 0.5 * dt;
//
//    auto x_it = body.posWorld.begin();
//    auto v_it = body.velocities.begin();
//    auto f_it = body.forces.begin();
//    auto m_it = body.masses.begin();
//    auto a_it = body.accels.begin();
//    for (; f_it != body.forces.end(); ++x_it, ++v_it, ++f_it, ++m_it, ++a_it) {
//        static Eigen::Vector3d g(0, Options::gravity(), 0);
//        auto a = *f_it / *m_it + g;
//        *x_it += dt * (*v_it + dt_2 * *a_it);
//        *v_it += dt_2 * (a + *a_it);
//        *a_it = a;
//    }
//}

}

void
World::addSoftBody(SoftBody* body)
{
    sBodies.push_back(body);
}

void
World::addObstacle(Obstacle* obstacle)
{
    sObstacles.push_back(obstacle);
}

void
World::step(double dt)
{
    for (auto& body : sBodies) {
        body->clearForces();
        body->computeInternalForces();

        // Integrate
        integrateExplicit(dt, *body);
        //integrateLeapfrog(dt, *body);

        // Collide
        for (auto o : sObstacles) {
            for (auto b : sBodies) {
                auto x_it = b->posWorld.begin();
                auto v_it = b->velocities.begin();
                for (uint32_t i = 0; i < b->size(); ++i, ++x_it, ++v_it) {
                    o->bounce(*x_it, *v_it);
                }
            }
        }
    }
}

const std::vector<const SoftBody*>
World::bodies()
{
    return std::vector<const SoftBody*>(sBodies.begin(), sBodies.end());
}

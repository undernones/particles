#include "World.h"
#include <physics/Obstacle.h>
#include <physics/SoftBody.h>
#include "Options.h"

std::vector<SoftBody*> World::sBodies;
std::vector<Obstacle*> World::sObstacles;

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
        auto x_it = body->posWorld.begin();
        auto v_it = body->velocities.begin();
        auto f_it = body->forces.begin();
        auto m_it = body->masses.begin();
        for (; f_it != body->forces.end(); ++x_it, ++v_it, ++f_it, ++m_it) {
            static Eigen::Vector3d g(0, Options::gravity(), 0);
            auto a = *f_it / *m_it + g;
            *v_it += dt * a;
            *x_it += dt * *v_it;
        }

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

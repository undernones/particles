#include "World.h"
#include <tbb/tbb.h>
#include <geom/Mesh.h>
#include <physics/Obstacle.h>
#include <physics/PlaneObstacle.h>
#include <physics/SoftBody.h>
#include "Options.h"

std::vector<SoftBody*> World::sBodies;
std::vector<Obstacle*> World::sObstacles;

namespace
{

class ExplicitIntegrator
{
public:
    ExplicitIntegrator(double dt, SoftBody& body)
        : mDt(dt)
        , mBody(body)
    {}

    void operator ()(const tbb::blocked_range<uint32_t> r) const
    {
        static Eigen::Vector3d g(0, Options::gravity(), 0);

        uint32_t lo = r.begin();
        uint32_t hi = r.end();

        auto x_it = mBody.posWorld.begin() + lo;
        auto v_it = mBody.velocities.begin() + lo;
        auto m_it = mBody.masses.begin() + lo;
        auto f_it = mBody.forces.begin() + lo;
        auto end = mBody.forces.begin() + hi;

        for (; f_it != end; ++x_it, ++v_it, ++f_it, ++m_it) {
            auto a = *f_it / *m_it + g;
            *v_it += mDt * a;
            *x_it += mDt * *v_it;
        }
    }

private:
    double mDt;
    SoftBody& mBody;
};

void
integrateExplicit(double dt, SoftBody& body)
{
    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, body.size()),
        ExplicitIntegrator(dt, body)
    );
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
World::init()
{
    Material material = {
        Options::mu(),
        Options::lambda(),
        Options::density(),
        Options::hardening(),
        Options::flowRate(),
        Options::yieldPoint(),
    };
    SoftBody* body = new SoftBody(Options::particleFile(), material);

    Mesh* mesh = new Mesh;
    if (Mesh::loadObj(Options::meshFile(), *mesh)) {
        body->setMesh(mesh);
    }

    addSoftBody(body);
    addObstacle(new PlaneObstacle(Eigen::Vector3d(0, 1, 0), -1, Options::friction()));
}

void
World::step(double dt)
{
    for (auto& body : sBodies) {
        body->updateState(dt);

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

        body->updateMesh();
    }
}

const std::vector<const SoftBody*>
World::bodies()
{
    return std::vector<const SoftBody*>(sBodies.begin(), sBodies.end());
}

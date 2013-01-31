#include "SoftBody.h"
#include <fstream>
#include <QtCore>
#include <geom/KdTree.h>
#include <geom/Mesh.h>
#include "kernels.h"
#include "../Utils.h" // TODO: dependencies!

#define PARALLEL

using Eigen::Vector3d;
using Eigen::Matrix3d;

namespace
{

bool
load(const std::string& filename, std::vector<Vector3d>& positions)
{
    std::ifstream file(filename.c_str());
    if (!file.is_open()) {
        return false;
    }

    Vector3d u;
    file >> u[0] >> u[1] >> u[2];
    while (file) {
        positions.push_back(u);
        file >> u[0] >> u[1] >> u[2];
    }

    return true;
}

template <typename T>
void
zero(std::vector<T>& list)
{
    for_each (list.begin(), list.end(), [](T& x) { x.setZero(); });
}

void
identity(SoftBody::MatrixList& matrices)
{
    for_each (
        matrices.begin(),
        matrices.end(),
        [](Matrix3d& m) { m.setIdentity(); }
    );
}

}

SoftBody::SoftBody(const std::string& positionsFile, const Material& material) :
    mBBox(),
    mMaterial(material),
    mMesh(nullptr)
{
    if (!load(positionsFile, posRest)) {
        return; // TODO: Throw exception?
    }

    uint32_t size = posRest.size();

    posWorld.resize(size);
    auto u = posRest.begin();
    for (auto& x : posWorld) {
        x = *u;
        ++u;
    }
    mBBox.add(posWorld);

    bases.resize(size);
    identity(bases);

    velocities.resize(size);
    zero(velocities);

    accels.resize(size);
    zero(accels);

    forces.resize(size);

    masses.resize(size);
    volumes.resize(size);
    radii.resize(size);

    neighborhoods.resize(size);

    defs.resize(size);
    identity(defs);

    strains.resize(size);
    zero(strains);

    stresses.resize(size);
    zero(stresses);

    updateNeighborhoods();
    updateRadii();
    updateRestQuantities();
}

SoftBody::~SoftBody()
{
}

void
SoftBody::setMesh(Mesh* mesh)
{
    Vector3d toTranslate = mBBox.center() - mesh->bbox().center();
    mesh->translate(toTranslate);

    mMesh = mesh;

    auto radius = avgRadius();
    auto kdTree = KdTree(posRest);

    mRestMesh.reserve(mesh->verts().size());
    mMeshNeighborhoods.reserve(mesh->verts().size());

    for (auto& v : mesh->verts()) {
        mRestMesh.push_back(v);

        mMeshNeighborhoods.push_back(Neighborhood());
        auto& hood = mMeshNeighborhoods.back();

        std::vector<uint32_t> indices;
        kdTree.neighbors(posRest, v, -1, radius, indices); // Get ALL neighbors
        for (auto j : indices) {
            auto distance = (posRest[j] - v).norm();
            auto weight = Kernels::standardkernel(radius, distance);
            hood.push_back(Neighbor(j, Vector3d(), weight));
        }
        hood.computeSum();
    }
}

void
SoftBody::updateMesh()
{
    if (mMesh == nullptr) return;

#ifdef PARALLEL
    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, mMesh->verts().size()),
        MeshProcessor(*this),
        tbb::simple_partitioner()
    );
#else
    updateMesh(0, mMesh->verts().size());
#endif
    //mMesh->updateNormals();
}

void
SoftBody::updateMesh(uint32_t lo, uint32_t hi)
{
    auto rest_it = mRestMesh.begin() + lo;
    auto hood_it = mMeshNeighborhoods.begin() + lo;
    auto vert_it = mMesh->verts().begin() + lo;

    auto end = mMesh->verts().begin() + hi;

    for (; vert_it != end; ++vert_it, ++rest_it, ++hood_it) {
        // compute normalized weighted average of neighbors' displacements.
        Vector3d displacement(0, 0, 0);
        for (auto& n : *hood_it) {
            displacement += n.w * (posWorld[n.index] - posRest[n.index]);
        }
        displacement /= hood_it->sum();

        // set v to rest_it + avg displacement.
        *vert_it = *rest_it + displacement;
    }
}

double
SoftBody::avgRadius() const
{
    double sum = 0;
    for (auto r : radii) {
        sum += r;
    }
    return sum / radii.size();
}

void
SoftBody::clearForces()
{
    zero(forces);
}

void
SoftBody::computeInternalForces()
{
#ifdef PARALLEL
    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, size()),
        ForceProcessor(*this),
        tbb::simple_partitioner()
    );
#else
    clearNeighborForces(0, size());
    computeFs(0, size());
    computeStrains(0, size());
    computeStresses(0, size());
    computeForces(0, size());
#endif
    accumulateForces();
}

void
SoftBody::updateNeighborhoods()
{
    auto kdTree = KdTree(posRest);

    std::vector<uint32_t> indices;
    indices.reserve(Neighborhood::MAX_SIZE + 1);

    auto neighbor_it = neighborhoods.begin();
    auto radius_it = radii.begin();
    uint32_t index = 0;
    for (auto& u : posRest) {
        //
        // A particle's neighborhood should not include itself. However, this
        // kdTree will return an index for the current particle. So increment
        // the neighbor count by 1, and then remove the "self" particle when
        // we're done.
        //

        kdTree.neighbors(posRest, u, Neighborhood::MAX_SIZE + 1, *radius_it, indices);
        auto selfLocation = std::find(indices.begin(), indices.end(), index);
        if (selfLocation != indices.end()) {
            indices.erase(selfLocation);
        }

        // If we find a neighbor we didn't already have, add it with an initial
        // weight of zero.
        //

        Neighborhood newNeighbors;

        for (auto j : indices) {
            if (neighbor_it->hasNeighbor(j)) {
                newNeighbors.push_back(neighbor_it->findNeighbor(j));
            } else {
                Vector3d u_ij = posRest[j] - u;
                Neighbor n(j, u_ij, 0.0);
                newNeighbors.push_back(n);
            }
        }
        *neighbor_it = newNeighbors;

        ++neighbor_it;
        ++radius_it;
        ++index;
    }
}

void
SoftBody::updateRadii()
{
    auto u_it = posRest.begin();
    auto n_it = neighborhoods.begin();
    auto r_it = radii.begin();
    for (; r_it != radii.end(); ++u_it, ++n_it, ++r_it) {
        double avg = 0;
        for (auto& n : *n_it) {
            avg += (*u_it - posRest[n.index]).norm();
        }
        avg /= n_it->size();
        *r_it = avg * 2.0;
    }
}

void
SoftBody::updateRestQuantities()
{
    auto x_it = posWorld.begin();
    auto u_it = posRest.begin();
    auto n_it = neighborhoods.begin();
    auto r_it = radii.begin();
    auto v_it = volumes.begin();
    auto m_it = masses.begin();

    for (auto& basis : bases) {

        basis.setZero();
        for (auto& n : *n_it) {
            double distance = (*u_it - posRest[n.index]).norm();
            n.w = Kernels::standardkernel(*r_it, distance);
            n.u = posRest[n.index] - *u_it;
            basis += n.u * n.u.transpose() * n.w;
        }
        *v_it = sqrt(basis.determinant() / Utils::cube(n_it->computeSum()));
        *m_it = mMaterial.density * *v_it;
        basis = basis.inverse().eval();

        ++x_it;
        ++u_it;
        ++n_it;
        ++r_it;
        ++v_it;
        ++m_it;
    }
}

void
SoftBody::clearNeighborForces(uint32_t lo, uint32_t hi)
{
    auto h_it = neighborhoods.begin() + lo;
    auto end = neighborhoods.begin() + hi;

    for (; h_it != end; ++h_it) {
        for (auto& n : *h_it) {
            n.f.setZero();
        }
    }
}

void
SoftBody::computeFs(uint32_t lo, uint32_t hi)
{
    auto n_it = neighborhoods.begin() + lo;
    auto f_it = defs.begin() + lo;
    auto u_it = posWorld.begin() + lo;
    auto b_it = bases.begin() + lo;

    auto end = posWorld.begin() + hi;

    Matrix3d rhs;
    for (; u_it != end; ++u_it, ++f_it, ++n_it, ++b_it) {
        rhs.setZero();
        for (auto& n : *n_it) {
            rhs += n.w * (posWorld[n.index] - *u_it) * n.u.transpose();
        }
        *f_it = rhs * *b_it;
    }
}

void
SoftBody::computeStrains(uint32_t lo, uint32_t hi)
{
    auto f_it = defs.begin() + lo;
    auto e_it = strains.begin() + lo;

    auto end = strains.begin() + hi;

    for (; e_it != end; ++e_it, ++f_it) {
        const Matrix3d& F = *f_it;
        // Linear Cauchy strain
        //e = 0.5 * (F + F.transpose()) - Matrix3d::Identity();
        // Quadratic Green strain
        *e_it = 0.5 * (F.transpose() * F - Matrix3d::Identity());
    }
}

void
SoftBody::computeStresses(uint32_t lo, uint32_t hi)
{
    auto f_it = defs.begin() + lo;
    auto e_it = strains.begin() + lo;
    auto s_it = stresses.begin() + lo;

    auto end = stresses.begin() + hi;

    for (; s_it != end; ++s_it, ++f_it, ++e_it) {
        const Matrix3d& e = *e_it;
        const Matrix3d& F = *f_it;

        double d = mMaterial.lambda * e.trace();
        Matrix3d diag;
        diag << d, 0, 0,
                0, d, 0,
                0, 0, d;
        *s_it = F * (2 * mMaterial.mu * e + diag); // TODO: Multiply by F?
    }
}

void
SoftBody::computeForces(uint32_t lo, uint32_t hi)
{
    auto vel_it = velocities.begin() + lo;
    auto v_it = volumes.begin() + lo;
    auto stress_it = stresses.begin() + lo;
    auto basis_it = bases.begin() + lo;
    auto n_it = neighborhoods.begin() + lo;
    auto f_it = forces.begin() + lo;

    auto end = forces.begin() + hi;

    for (; f_it != end; ++f_it, ++vel_it, ++v_it, ++stress_it, ++basis_it, ++n_it) {
        double volume = *v_it;
        const Matrix3d& stress = *stress_it;
        const Matrix3d& basis = *basis_it;
        Neighborhood& neighborhood = *n_it;

        Matrix3d Fe = -2 * volume * stress * basis;
        for (auto& n : neighborhood) {
            Vector3d force = Fe * (n.u * n.w);
            n.f += force;
            *f_it -= force;

            Vector3d viscous = volume * (velocities[n.index] - *vel_it) * n.w;
            n.f -= viscous;
            *f_it += viscous;
        }
    }
}

void
SoftBody::accumulateForces()
{
    for (auto& hood : neighborhoods) {
        for (auto& n : hood) {
            forces[n.index] += n.f;
        }
    }
}


#include "SoftBody.h"
#include <fstream>
#include <geom/KdTree.h>
#include "kernels.h"
#include "../Utils.h" // TODO: dependencies!
#include <iostream>

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
    mMaterial(material)
{
    if (!load(positionsFile, mPosRest)) {
        return; // TODO: Throw exception?
    }

    uint32_t size = mPosRest.size();

    mPosWorld.resize(size);
    auto u = mPosRest.begin();
    for (auto& x : mPosWorld) {
        x = *u;
        //x[0] *= 2;
        ++u;
    }
    
    mBases.resize(size);
    identity(mBases);

    mVelocities.resize(size);
    zero(mVelocities);

    mForces.resize(size);

    mMasses.resize(size);
    mVolumes.resize(size);
    mRadii.resize(size);

    mNeighborhoods.resize(size);

    mDefs.resize(size);
    identity(mDefs);

    mStrains.resize(size);
    zero(mStrains);

    mStresses.resize(size);
    zero(mStresses);

    updateNeighborhoods();
    updateRadii();
    updateRestQuantities();
}

SoftBody::~SoftBody()
{
}

void
SoftBody::clearForces()
{
    zero(mForces);
}

void
SoftBody::computeInternalForces()
{
    computeFs();
    computeStrains();
    computeStresses();
    computeForces();
}

void
SoftBody::updateNeighborhoods()
{
    auto kdTree = KdTree(mPosRest);

    std::vector<uint32_t> indices;
    indices.reserve(Neighborhood::MAX_SIZE + 1);

    auto neighbor_it = mNeighborhoods.begin();
    auto radius_it = mRadii.begin();
    uint32_t index = 0;
    for (auto& u : mPosRest) {
        //
        // A particle's neighborhood should not include itself. However, this
        // kdTree will return an index for the current particle. So increment
        // the neighbor count by 1, and then remove the "self" particle when
        // we're done.
        //

        kdTree.neighbors(mPosRest, u, Neighborhood::MAX_SIZE + 1, *radius_it, indices);
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
                Vector3d u_ij = mPosRest[j] - u;
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
    auto u_it = mPosRest.begin();
    auto n_it = mNeighborhoods.begin();
    auto r_it = mRadii.begin();
    for (; r_it != mRadii.end(); ++u_it, ++n_it, ++r_it) {
        double avg = 0;
        for (auto& n : *n_it) {
            avg += (*u_it - mPosRest[n.index]).norm();
        }
        avg /= n_it->size();
        *r_it = avg * 2.0;
    }
}

void
SoftBody::updateRestQuantities()
{
    auto x_it = mPosWorld.begin();
    auto u_it = mPosRest.begin();
    auto n_it = mNeighborhoods.begin();
    auto r_it = mRadii.begin();
    auto v_it = mVolumes.begin();
    auto m_it = mMasses.begin();

    for (auto& basis : mBases) {

        double wSum = 0;
        basis.setZero();
        for (auto& n : *n_it) {
            double distance = (*u_it - mPosRest[n.index]).norm();
            n.w = Kernels::standardkernel(*r_it, distance);
            n.u = mPosRest[n.index] - *u_it;
            basis += n.u * n.u.transpose() * n.w;
            wSum += n.w;
        }
        *v_it = sqrt(basis.determinant() / Utils::cube(wSum));
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
SoftBody::computeFs()
{
    Matrix3d rhs;
    auto n_it = mNeighborhoods.begin();
    auto f_it = mDefs.begin();
    auto u_it = mPosWorld.begin();
    auto b_it = mBases.begin();
    for (; u_it != mPosWorld.end(); ++u_it, ++f_it, ++n_it, ++b_it) {
        rhs.setZero();
        for (auto& n : *n_it) {
            rhs += n.w * (mPosWorld[n.index] - *u_it) * n.u.transpose();
        }
        *f_it = rhs * *b_it;
    }
}

void
SoftBody::computeStrains()
{
    auto f_it = mDefs.begin();
    for (auto& e : mStrains) {
        const Matrix3d& F = *f_it;
        // Linear Cauchy strain
        //e = 0.5 * (F + F.transpose()) - Matrix3d::Identity();
        // Quadratic Green strain
        e = 0.5 * (F.transpose() * F - Matrix3d::Identity());
    }
}

void
SoftBody::computeStresses()
{
    auto f_it = mDefs.begin();
    auto e_it = mStrains.begin();
    for (auto& stress : mStresses) {
        const Matrix3d& e = *e_it;
        const Matrix3d& F = *f_it;

        double d = mMaterial.lambda * e.trace();
        Matrix3d diag;
        diag << d, 0, 0,
                0, d, 0,
                0, 0, d;
        stress = F * (2 * mMaterial.mu * e + diag); // TODO: Multiply by F?

        ++f_it;
        ++e_it;
    }
}

void
SoftBody::computeForces()
{
    auto v_it = mVolumes.begin();
    auto stress_it = mStresses.begin();
    auto basis_it = mBases.begin();
    auto n_it = mNeighborhoods.begin();

    for (auto& f : mForces) {
        double volume = *v_it;
        const Matrix3d& stress = *stress_it;
        const Matrix3d& basis = *basis_it;
        const Neighborhood& neighborhood = *n_it;

        Matrix3d Fe = -volume * stress * basis;
        for (auto& n : neighborhood) {
            Vector3d force = Fe * (n.u * n.w);
            mForces[n.index] += force;
            f -= force;
        }

        ++v_it;
        ++stress_it;
        ++basis_it;
        ++n_it;
    }
}


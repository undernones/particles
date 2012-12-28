#include "SoftBody.h"
#include <fstream>
#include <geom/KdTree.h>
#include "kernels.h"

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

SoftBody::SoftBody(const std::string& positionsFile, const Material& material)
{
    if (!load(positionsFile, mPosRest)) {
        return; // TODO: Throw exception?
    }

    uint32_t size = mPosRest.size();

    mPosWorld.resize(size);
    auto u = mPosRest.begin();
    for (auto& x : mPosWorld) {
        x = *u;
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
SoftBody::updateRestQuantities()
{
    auto x_it = mPosWorld.begin();
    auto u_it = mPosRest.begin();
    auto n_it = mNeighborhoods.begin();
    auto r_it = mRadii.begin();
    for (auto& basis : mBases) {
        basis.setZero();
        for (auto& neighbor : *n_it) {
            double distance = (*u_it - mPosRest[neighbor.index]).norm();
            neighbor.w = Kernels::standardkernel(*r_it, distance);
        }



        /*
    BOOST_FOREACH (Particle &p, mParticles){
        BOOST_FOREACH (Neighbor &n, p.neighbors){
            n.w_ij = Kernels::standardkernel(p.radius, mag(n.u));
        */
        ++x_it;
        ++u_it;
        ++n_it;
        ++r_it;
    }
}

void
SoftBody::computeFs()
{
}

void
SoftBody::computeStrains()
{
}

void
SoftBody::computeStresses()
{
}

void
SoftBody::computeForces()
{
}


#include "SoftBody.h"
#include <fstream>
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

    mNeighborhoods.resize(size);
    for (auto& n : mNeighborhoods) {
        n.reserve(16);
    }

    mDefs.resize(size);
    identity(mDefs);

    mStrains.resize(size);
    zero(mStrains);

    mStresses.resize(size);
    zero(mStresses);
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


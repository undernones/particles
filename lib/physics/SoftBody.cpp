#include "SoftBody.h"
#include <fstream>

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

    mPosWorld.resize(mPosRest.size());
    auto u = mPosRest.begin();
    for (auto& x : mPosWorld) {
        x = *u;
    }
    
    mMasses.resize(mPosRest.size());

    mVelocities.resize(mPosRest.size());
    zero(mVelocities);

    mForces.resize(mPosRest.size());

    mNeighborhoods.resize(mPosRest.size());
    for (auto& n : mNeighborhoods) {
        n.resize(16);
    }

    mDefs.resize(mPosRest.size());
    identity(mDefs);

    mStrains.resize(mPosRest.size());
    zero(mStrains);

    mStresses.resize(mPosRest.size());
    zero(mStresses);
}

SoftBody::~SoftBody()
{
}


#ifndef PHYSICS_SOFTBODY_H
#define PHYSICS_SOFTBODY_H

#include <vector>
#include <Eigen>
#include "Material.h"

class SoftBody
{
public:
    typedef std::vector<Eigen::Vector3d> VectorList;
    typedef std::vector<Eigen::Matrix3d> MatrixList;

    SoftBody(const std::string& positionsFile, const Material& material);
    ~SoftBody();

    const MatrixList& defs() const { return mDefs; }
    MatrixList& defs() { return mDefs; }

    const MatrixList& strains() const { return mStrains; }
    MatrixList& strains() { return mStrains; }

    const MatrixList& stresses() const { return mStresses; }
    MatrixList& stresses() { return mStresses; }

    inline const Material& material() const { return mMaterial; }

    void clearForces();
    void computeInternalForces();

private:
    VectorList mMasses;
    VectorList mPosWorld;
    VectorList mPosRest;
    VectorList mVelocities;
    VectorList mForces;
    std::vector< std::vector<uint32_t> > mNeighborhoods;
    MatrixList mDefs;
    MatrixList mStrains;
    MatrixList mStresses;

    Material mMaterial;

    SoftBody(const SoftBody&);
    SoftBody& operator =(const SoftBody&);

    void computeFs();
    void computeStrains();
    void computeStresses();
    void computeForces();
};

#endif // PHYSICS_SOFTBODY_H

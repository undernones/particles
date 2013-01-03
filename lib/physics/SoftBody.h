#ifndef PHYSICS_SOFTBODY_H
#define PHYSICS_SOFTBODY_H

#include <vector>
#include <Eigen>
#include <geom/Neighborhood.h>
#include "Material.h"

class SoftBody
{
public:
    typedef std::vector<Eigen::Vector3d> VectorList;
    typedef std::vector<Eigen::Matrix3d> MatrixList;

    MatrixList bases;
    VectorList posWorld;
    VectorList posRest;
    VectorList velocities;
    VectorList forces;
    std::vector<double> masses;
    std::vector<double> volumes;
    std::vector<double> radii;
    std::vector<Neighborhood> neighborhoods;
    MatrixList defs;
    MatrixList strains;
    MatrixList stresses;

    SoftBody(const std::string& positionsFile, const Material& material);
    ~SoftBody();

    inline const Material& material() const { return mMaterial; }

    void clearForces();
    void computeInternalForces();
    void updateNeighborhoods();
    void updateRadii();
    void updateRestQuantities();

    inline size_t size() const { return bases.size(); }

private:
    Material mMaterial;

    SoftBody(const SoftBody&);
    SoftBody& operator =(const SoftBody&);

    void computeFs();
    void computeStrains();
    void computeStresses();
    void computeForces();
};

#endif // PHYSICS_SOFTBODY_H

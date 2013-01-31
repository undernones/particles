#ifndef PHYSICS_SOFTBODY_H
#define PHYSICS_SOFTBODY_H

#include <vector>
#include <Eigen>
#include <geom/BBox.h>
#include <geom/Neighborhood.h>
#include "Material.h"

class Mesh;
class SoftBody
{
public:
    typedef std::vector<Eigen::Vector3d> VectorList;
    typedef std::vector<Eigen::Matrix3d> MatrixList;

    MatrixList bases;
    VectorList posWorld;
    VectorList posRest;
    VectorList velocities;
    VectorList accels;
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

    // Don't call setMesh() until after the points have been loaded.
    void setMesh(Mesh* mesh);
    const Mesh* mesh() const { return mMesh; }
    bool hasMesh() const { return mMesh != nullptr; }
    void updateMesh();

private:
    class ForceProcessor
    {
    public:
        ForceProcessor(SoftBody& body)
            : mBody(body)
        {}

        void operator ()(uint32_t i)
        {
            mBody.clearNeighborForces(i, i + 1);
            mBody.computeFs(i, i + 1);
            mBody.computeStrains(i, i + 1);
            mBody.computeStresses(i, i + 1);
            mBody.computeForces(i, i + 1);
        }

    private:
        SoftBody& mBody;
    };
    // ----------------------------------------------------------------------

    class MeshProcessor
    {
    public:
        MeshProcessor(SoftBody& body)
            : mBody(body)
        {}

        void operator ()(uint32_t i)
        {
            mBody.updateMesh(i, i + 1);
        }

    private:
        SoftBody& mBody;
    };
    // ----------------------------------------------------------------------

    BBox mBBox;
    Material mMaterial;

    Mesh* mMesh;
    VectorList mRestMesh;
    std::vector<Neighborhood> mMeshNeighborhoods;

    std::vector<uint32_t> mPointIndices;
    std::vector<uint32_t> mMeshIndices;

    SoftBody(const SoftBody&);
    SoftBody& operator =(const SoftBody&);

    void clearNeighborForces(uint32_t lo, uint32_t hi);
    void computeFs(uint32_t lo, uint32_t hi);
    void computeStrains(uint32_t lo, uint32_t hi);
    void computeStresses(uint32_t lo, uint32_t hi);
    void computeForces(uint32_t lo, uint32_t hi);
    void accumulateForces(); // Not parallelizable

    void updateMesh(uint32_t lo, uint32_t hi);

    double avgRadius() const;
};

#endif // PHYSICS_SOFTBODY_H

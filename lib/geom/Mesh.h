#ifndef GEOM_MESH_H
#define GEOM_MESH_H

#include <string>
#include <vector>
#include <Eigen>
#include "BBox.h"
#include "Triangle.h"

class Mesh
{
public:
    static std::string makeObjName(const std::string& dir, uint32_t frameNum);

    static bool loadObj(const std::string& filename, Mesh& mesh);
    bool saveObj(const std::string& filename) const;

    const std::vector<Eigen::Vector3d>& normals() const { return mNormals; }
    const std::vector<Eigen::Vector3d>& verts() const { return mVerts; }
    const std::vector<Triangle>& faces() const { return mFaces; }
    const BBox& bbox() const { return mBBox; }

    std::vector<Eigen::Vector3d>& verts() { return mVerts; }

    void updateNormals();

    void scale(double k);
    void translate(const Eigen::Vector3d& v);

private:
    std::vector<Eigen::Vector3d> mNormals;
    std::vector<Eigen::Vector3d> mVerts;
    std::vector<Triangle> mFaces;
    BBox mBBox;
};

#endif // GEOM_MESH_H

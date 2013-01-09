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
    static bool loadObj(const std::string& filename, Mesh& mesh);

    const std::vector<Eigen::Vector3d>& normals() const { return mNormals; }
    const std::vector<Eigen::Vector3d>& verts() const { return mVerts; }
    const std::vector<Triangle>& faces() const { return mFaces; }
    const BBox& bbox() const { return mBBox; }

private:
    std::vector<Eigen::Vector3d> mNormals;
    std::vector<Eigen::Vector3d> mVerts;
    std::vector<Triangle> mFaces;
    BBox mBBox;
};

#endif // GEOM_MESH_H

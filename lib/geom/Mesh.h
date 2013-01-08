#ifndef GEOM_MESH_H
#define GEOM_MESH_H

#include <string>
#include <vector>
#include <Eigen>
#include "Triangle.h"

class Mesh
{
public:
    std::vector<Eigen::Vector3d> verts;
    std::vector<Triangle> faces;

    static Mesh loadObj(const std::string& filename);
};

#endif // GEOM_MESH_H

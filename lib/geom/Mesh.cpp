#include "Mesh.h"
#include <istream>
#include <fstream>
#include <iostream>

using Eigen::Vector3d;

namespace
{

std::string nextLine(std::istream& in)
{
    std::string line;
    while (std::getline(in, line)) {
        if (line.length() > 0 && line[0] != '#') {
            return line;
        }
    }
    return std::string();
}

}

bool
Mesh::loadObj(const std::string& filename, Mesh& mesh)
{
    uint32_t vertCount = 0;
    uint32_t faceCount = 0;
    bool hasNormals = false;
    bool hasTextures = false;

    std::string line;
    {
        std::ifstream inf(filename.c_str(), std::ios::in);
        if (!inf.is_open()) return false;

        while ((line = nextLine(inf)).length() > 0) {
            switch (line[0]) {
            case 'v':
                if (line[1] == ' ' || line[1] == '\t') vertCount++;
                else if (line[1] == 'n') hasNormals = true;
                else if (line[1] == 't') hasTextures = true;
                else { return false; }
                break;

            case 'f':
                faceCount++;
                break;

            default:
                // Do nothing
                break;
            }
        }
    }

    {
        mesh.verts.reserve(vertCount);
        mesh.faces.reserve(faceCount);
        mesh.normals.reserve(vertCount);

        std::ifstream inf(filename.c_str(), std::ios::in);
        if (!inf.is_open()) return false;

        double x, y, z;
        uint32_t p, q, r;
        while ((line = nextLine(inf)).length() > 0) {
            switch (line[0]) {
            case 'v':
                if (line[1] == ' ' || line[1] == '\t') {
                    std::stringstream stream(line.substr(2));
                    stream >> x >> y >> z;
                    mesh.verts.push_back(Vector3d(x, y, z));
                }
                break;

            case 'f':
                if (hasNormals && hasTextures) {
                } else if (hasNormals) {
                } else if (hasTextures) {
                } else {
                    std::stringstream stream(line.substr(1));
                    stream >> p >> q >> r;
                    p--; q--; r--;
                    mesh.faces.push_back(Triangle(p, q, r));

                    Vector3d v1 = mesh.verts[q] - mesh.verts[p];
                    Vector3d v2 = mesh.verts[r] - mesh.verts[p];
                    mesh.normals.push_back(v1.cross(v2).normalized());
                }
                break;

            default:
                // Do nothing
                break;
            }
        }
    }

    return true;
}

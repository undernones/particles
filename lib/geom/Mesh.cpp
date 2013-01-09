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

Mesh
Mesh::loadObj(const std::string& filename)
{
    Mesh result;

    uint32_t vertCount = 0;
    uint32_t faceCount = 0;
    bool hasNormals = false;
    bool hasTextures = false;

    std::string line;
    {
        std::ifstream inf(filename.c_str(), std::ios::in);
        while ((line = nextLine(inf)).length() > 0) {
            switch (line[0]) {
            case 'v':
                if (line[1] == ' ' || line[1] == '\t') vertCount++;
                else if (line[1] == 'n') hasNormals = true;
                else if (line[1] == 't') hasTextures = true;
                else { /* TODO: throw exception? */ }
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
        result.verts.reserve(vertCount);
        result.faces.reserve(faceCount);

        double x, y, z;
        uint32_t p, q, r;
        std::ifstream inf(filename.c_str(), std::ios::in);
        while ((line = nextLine(inf)).length() > 0) {
            switch (line[0]) {
            case 'v':
                if (line[1] == ' ' || line[1] == '\t') {
                    std::stringstream stream(line.substr(2));
                    stream >> x >> y >> z;
                    result.verts.push_back(Vector3d(x, y, z));
                }
                break;

            case 'f':
                if (hasNormals && hasTextures) {
                } else if (hasNormals) {
                } else if (hasTextures) {
                } else {
                    std::stringstream stream(line.substr(1));
                    stream >> p >> q >> r;
                }
                result.faces.push_back(Triangle(p-1, q-1, r-1));
                break;

            default:
                // Do nothing
                break;
            }
        }
    }

    return result;
}

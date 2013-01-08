#include "Mesh.h"
#include <istream>
#include <fstream>
#include <iostream>

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
                if (line[1] == ' ') vertCount++;
                else if (line[1] == 'n') hasNormals = true;
                else if (line[1] == 't') hasTextures = true;
                else { /* TODO: throw exception? */ }
                break;

            case 'f':
                faceCount++;
                break;
            }
        }
    }

    {
        result.verts.resize(vertCount);
        result.faces.resize(faceCount);

        std::ifstream inf(filename.c_str(), std::ios::in);
        while ((line = nextLine(inf)).length() > 0) {

        }
    }

    return result;
}

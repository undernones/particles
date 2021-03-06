#include "Mesh.h"
#include <fstream>

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

std::string
Mesh::makeObjName(const std::string& dir, uint32_t frameNum)
{
    static char result[1024];
    std::string format(dir + "/surface.%06u.obj");
    sprintf(result, format.c_str(), frameNum);
    return std::string(result);
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
        mesh.mVerts.reserve(vertCount);
        mesh.mFaces.reserve(faceCount);
        mesh.mNormals.reserve(vertCount);

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
                    mesh.mVerts.push_back(Vector3d(x, y, z));
                    mesh.mBBox.add(mesh.mVerts.back());
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
                    mesh.mFaces.push_back(Triangle(p, q, r));

                    Vector3d v1 = mesh.mVerts[q] - mesh.mVerts[p];
                    Vector3d v2 = mesh.mVerts[r] - mesh.mVerts[p];
                    mesh.mNormals.push_back(v1.cross(v2).normalized());
                }
                break;

            default:
                // Do nothing
                break;
            }
        }
    }

    return mesh.mVerts.size() == vertCount
        && mesh.mFaces.size() == faceCount
        ;
}

bool
Mesh::saveObj(const std::string& filename) const
{
    std::ofstream outf(filename.c_str(), std::ios::out);
    if (!outf.is_open()) return false;

    for (auto& v : mVerts) {
        outf << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }

    for (auto& f : mFaces) {
        outf << "f " << f[0]+1 << " " << f[1]+1 << " " << f[2]+1 << std::endl;
    }

    return outf.good();
}

void
Mesh::updateNormals()
{
    auto face_it = mFaces.begin();
    for (auto& n : mNormals) {
        uint32_t p = (*face_it)[0];
        uint32_t q = (*face_it)[1];
        uint32_t r = (*face_it)[2];
        Vector3d v1 = mVerts[q] - mVerts[p];
        Vector3d v2 = mVerts[r] - mVerts[p];
        n = v1.cross(v2).normalized();
        ++face_it;
    }
}

void
Mesh::scale(double k)
{
    for (auto& vert : mVerts) {
        vert *= k;
    }
    mBBox.lo *= k;
    mBBox.hi *= k;
}

void
Mesh::translate(const Eigen::Vector3d& v)
{
    for (auto& vert : mVerts) {
        vert += v;
    }
    mBBox.lo += v;
    mBBox.hi += v;
}

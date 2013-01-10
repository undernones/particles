#include <iostream>
#include <vector>
#include <Eigen>
#include <geom/Triangle.h>

namespace
{

std::vector<Eigen::Vector3d> verts;
std::vector<Triangle> faces;

}

int
main(int argc, char* argv[])
{
    if (argc < 9) {
        std::cerr << "Usage: " << std::endl
                  << argv[0]
                  << " lo_x lo_y lo_z hi_x hi_y hi_z thickness fold_count"
                  << std::endl;
        return 1;
    }

    const double lox = atof(argv[1]);
    const double loy = atof(argv[2]);
    const double loz = atof(argv[3]);

    const double hix = atof(argv[4]);
    const double hiy = atof(argv[5]);
    const double hiz = atof(argv[6]);

    const double thickness = atof(argv[7]);
    const unsigned fold_count = atoi(argv[8]);

    std::cerr
        << lox << ", " << loy << ", " << loz << std::endl
        << hix << ", " << hiy << ", " << hiz << std::endl
        << thickness << std::endl
        << fold_count << std::endl;

    for (unsigned i = 0; i <= fold_count; ++i) {
        double y = i * (hiy - loy) / fold_count + loy;
        double xlo = lox;
        double xhi = hix;
        double zlo = loz;
        double zhi = hiz;
        if (i % 2) {
            xlo += thickness;
            xhi -= thickness;
            zlo += thickness;
            zhi -= thickness;
        }
        verts.push_back(Eigen::Vector3d(xlo, y, zlo));
        verts.push_back(Eigen::Vector3d(xlo, y, zhi));
        verts.push_back(Eigen::Vector3d(xhi, y, zhi));
        verts.push_back(Eigen::Vector3d(xhi, y, zlo));
    }

    faces.push_back(Triangle(1, 3, 2));
    faces.push_back(Triangle(1, 4, 3));
    unsigned base;
    for (unsigned i = 0; i < fold_count; ++i) {
        base = i * 4 + 1;
        faces.push_back(Triangle(base + 0, base + 1, base + 4));
        faces.push_back(Triangle(base + 4, base + 1, base + 5));
        faces.push_back(Triangle(base + 1, base + 2, base + 5));
        faces.push_back(Triangle(base + 5, base + 2, base + 6));
        faces.push_back(Triangle(base + 2, base + 3, base + 6));
        faces.push_back(Triangle(base + 6, base + 3, base + 7));
        faces.push_back(Triangle(base + 3, base + 0, base + 7));
        faces.push_back(Triangle(base + 7, base + 0, base + 4));
    }
    base += 4;
    faces.push_back(Triangle(base + 0, base + 1, base + 2));
    faces.push_back(Triangle(base + 0, base + 2, base + 3));

    for (auto& v : verts) {
        std::cout << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }
    for (auto& f : faces) {
        std::cout << "f " << f[0] << " " << f[1] << " " << f[2] << std::endl;
    }

    return 0;
}

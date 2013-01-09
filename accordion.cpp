#include <iostream>

int
main(int argc, char* argv[])
{
    if (argc < 9) {
        std::cerr << "Usage: " << std::endl
                  << argv[0]
                  << " lo_x lo_y lo_z hi_x hi_y hi_z thickness fold_count"
                  << std::endl;
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
        double y = i * (hiz - loz) / fold_count;
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
        std::cout
            << "v " << xlo << " " << y << " " << zlo << std::endl
            << "v " << xlo << " " << y << " " << zhi << std::endl
            << "v " << xhi << " " << y << " " << zhi << std::endl
            << "v " << xhi << " " << y << " " << zlo << std::endl
        ;
    }

    return 0;
}

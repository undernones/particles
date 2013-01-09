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

    double lox = atof(argv[1]);
    double loy = atof(argv[2]);
    double loz = atof(argv[3]);

    double hix = atof(argv[4]);
    double hiy = atof(argv[5]);
    double hiz = atof(argv[6]);

    double thickness = atof(argv[7]);
    unsigned fold_count = atoi(argv[8]);

    std::cerr
        << lox << ", " << loy << ", " << loz << std::endl
        << hix << ", " << hiy << ", " << hiz << std::endl
        << thickness << std::endl
        << fold_count << std::endl;

    for (unsigned i = 0; i <= fold_count; ++i) {
        double y = i * (hiz - loz) / fold_count;
        std::cout << lox << " " << y << " " << loz << std::endl;
    }

    return 0;
}

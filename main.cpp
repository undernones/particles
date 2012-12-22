#include <iostream>
#include <stdexcept>
#include "Options.h"
#include "Utils.h"

namespace
{
const int SUCCESS = 0;
const int ERROR = 1;
}

int
main(int argc, char* argv[])
{
    try {
        Options::init(argc, argv);

        uint32_t totalSteps = Options::duration() / Options::dt();
        std::cout << "Duration:    " << Options::duration() << "s" << std::endl
                  << "Timestep:    " << Options::dt() << "s" << std::endl
                  << "Total steps: " << totalSteps << std::endl
                  ;

        for (uint32_t i = 0; i < totalSteps; ++i) {
            //World::step(Options::dt());
        }
    } catch (std::exception &e) {
        Utils::error(std::string("Exception: ") + e.what());
        return ERROR;
    }

    return SUCCESS;
}

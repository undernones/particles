#ifndef WORLD_H
#define WORLD_H

#include <vector>

class SoftBody;
class World
{
public:
    static void step(double dt);
    static void addSoftBody(SoftBody* body);

private:
    static std::vector<SoftBody*> sBodies;
};

#endif // WORLD_H

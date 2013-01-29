#ifndef WORLD_H
#define WORLD_H

#include <vector>

class Obstacle;
class SoftBody;
class World
{
public:
    static void init();

    static void step(double dt);
    static void addSoftBody(SoftBody* body);
    static void addObstacle(Obstacle* obstacle);

    static const std::vector<const SoftBody*> bodies();

private:
    static std::vector<SoftBody*> sBodies;
    static std::vector<Obstacle*> sObstacles;
};

#endif // WORLD_H

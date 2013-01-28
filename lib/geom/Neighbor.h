#ifndef GEOM_NEIGHBOR_H
#define GEOM_NEIGHBOR_H

#include <Eigen>

struct Neighbor
{
    static const uint32_t INVALID;

    uint32_t index;
    Eigen::Vector3d u;
    Eigen::Vector3d f;
    double w;

    Neighbor();
    Neighbor(uint32_t index, const Eigen::Vector3d& u, double w);
};

#endif // GEOM_NEIGHBOR_H

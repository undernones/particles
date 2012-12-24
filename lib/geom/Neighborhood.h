#ifndef GEOM_NEIGHBORHOOD_H
#define GEOM_NEIGHBORHOOD_H

#include "Neighbor.h"
#include "Collection.h"

class Neighborhood : public Collection<Neighbor>
{
public:
    static const uint32_t MAX_SIZE;

    explicit Neighborhood();

    bool hasNeighbor(uint32_t j);
    Neighbor findNeighbor(uint32_t j);
};

#endif // GEOM_NEIGHBORHOOD_H

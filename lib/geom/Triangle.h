#ifndef GEOM_TRIANGLE_H
#define GEOM_TRIANGLE_H

#include <stdint.h>

class Triangle
{
public:
    uint32_t indices[3];

    Triangle(uint32_t x, uint32_t y, uint32_t z);

    inline uint32_t& operator[](uint32_t i) { return indices[i]; }
    inline uint32_t  operator[](uint32_t i) const { return indices[i]; }

    inline void set(uint32_t x, uint32_t y, uint32_t z)
    {
        indices[0] = x;
        indices[1] = y;
        indices[2] = z;
    }
};

#endif // GEOM_TRIANGLE_H

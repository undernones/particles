#include "Neighbor.h"

const uint32_t Neighbor::INVALID = std::numeric_limits<uint32_t>::max();

Neighbor::Neighbor() : index(INVALID)
{
    index = INVALID;
}

Neighbor::Neighbor(uint32_t index_, const Eigen::Vector3d& u_, double w_) :
    index(index_),
    u(u_),
    w(w_)
{
}

#include "Octree.h"

using Eigen::Vector3d;

Octree::Node::Node(Octree& owner, const Eigen::Vector3d& lo, const Eigen::Vector3d& hi)
:   tree(owner)
,   isExternal(true)
,   min(lo)
,   max(hi)
{
}

Octree::Node::~Node()
{
    for (uint32_t i = 0; i < 8; ++i) {
        if (children[i] != nullptr) {
            delete children[i];
        }
    }
}

Vector3d
Octree::Node::center() const
{
    return (0.5 * (max - min)) + min;
}

void
Octree::Node::split()
{
}

// --------------------------------------------------------------------------

Octree::Octree(const Mesh& mesh, uint32_t maxDepth)
:   mHead(nullptr)
{
}

Octree::~Octree()
{
}

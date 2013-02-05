#ifndef GEOM_OCTREE_H
#define GEOM_OCTREE_H

#include <vector>
#include <Eigen>

class Mesh;
class Octree
{
public:
    Octree(const Mesh& mesh, uint32_t maxDepth);
    ~Octree();

private:
    class Node
    {
    public:
        Node(Octree& owner, const Eigen::Vector3d& lo, const Eigen::Vector3d& hi);
        ~Node();

        inline bool isLeaf() const
        {
            // If it has one, it has them all.
            return children[0] == nullptr;
        }

        Eigen::Vector3d center() const;
        void split();

        Octree& tree;
        bool isExternal;
        const Eigen::Vector3d& min;
        const Eigen::Vector3d& max;
        Node* children[8];
    };

    Node* mHead;
    std::vector<Eigen::Vector3d> mPoints;
};

#endif // GEOM_OCTREE_H

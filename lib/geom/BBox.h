#ifndef GEOM_BBOX_H
#define GEOM_BBOX_H

#include <Eigen>
#include <vector>

class BBox
{
public:
    Eigen::Vector3d lo;
    Eigen::Vector3d hi;

    BBox();

    bool contains(const Eigen::Vector3d& p) const;
    void add(const BBox& other);
    void add(const Eigen::Vector3d& p);
    void add(const std::vector<Eigen::Vector3d>& points);

    Eigen::Vector3d center() const;
};

#endif // GEOM_BBOX_H

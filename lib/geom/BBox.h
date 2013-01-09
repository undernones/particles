#ifndef GEOM_BBOX_H
#define GEOM_BBOX_H

#include <Eigen>

class BBox
{
public:
    Eigen::Vector3d lo;
    Eigen::Vector3d hi;

    BBox();

    bool contains(const Eigen::Vector3d& p) const;
    BBox add(const BBox& other) const;
    BBox add(const Eigen::Vector3d& p) const;
};

#endif // GEOM_BBOX_H

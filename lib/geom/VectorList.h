#ifndef GEOM_VECTORLIST_H
#define GEOM_VECTORLIST_H

#include <iostream>
#include <Eigen>
#include "Collection.h"

class VectorList : public Collection<Eigen::Vector3d>
{
public:
    VectorList();
    VectorList(size_t n);
    VectorList(const VectorList& other);
    ~VectorList();

    void setZero();
    double dot(const VectorList& other) const;
};

VectorList& operator +=(VectorList& lhs, const VectorList& rhs);
VectorList operator +(const VectorList& lhs, const VectorList& rhs);
VectorList& operator -=(VectorList& lhs, const VectorList& rhs);
VectorList operator -(const VectorList& lhs, const VectorList& rhs);
VectorList operator *(double k, const VectorList& rhs);

std::ostream& operator <<(std::ostream& stream, const VectorList& list);

#endif // GEOM_VECTORLIST_H

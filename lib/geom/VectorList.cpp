#include "VectorList.h"

VectorList::VectorList() : Collection<Eigen::Vector3d>()
{
}

VectorList::VectorList(size_t n) : Collection<Eigen::Vector3d>(n)
{
}

VectorList::VectorList(const VectorList& other) : Collection<Eigen::Vector3d>(other)
{
}

VectorList::~VectorList()
{
}

VectorList
operator +(const VectorList& lhs, const VectorList& rhs)
{
    assert(lhs.size() == rhs.size());

    VectorList result;
    result.reserve(lhs.size());

    for (size_t i = 0; i < lhs.size(); ++i) {
        result.push_back(lhs[i] + rhs[i]);
    }
    return result;
}

VectorList
operator -(const VectorList& lhs, const VectorList& rhs)
{
    assert(lhs.size() == rhs.size());

    VectorList result;
    result.reserve(lhs.size());

    for (size_t i = 0; i < lhs.size(); ++i) {
        result.push_back(lhs[i] - rhs[i]);
    }
    return result;
}

VectorList
operator *(double k, const VectorList& rhs)
{
    VectorList result;
    result.reserve(rhs.size());

    for (auto& r : rhs) {
        result.push_back(k * r);
    }
    return result;
}

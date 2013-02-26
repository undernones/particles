#include "VectorList.h"
#include <tbb/tbb.h>
#include <QtCore>

using Eigen::Vector3d;

namespace
{

struct Combiner
{
public:
    Combiner(
        const VectorList& lhs,
        const VectorList& rhs,
        VectorList& result,
        const std::function<Vector3d (const Vector3d&, const Vector3d&)>& operation
    )
        : mIsScalar(false)
        , mScalar(0)
        , mLeft(lhs)
        , mRight(rhs)
        , mResult(result)
        , mVectorOp(operation)
    {
        assert(lhs.size() == rhs.size());
        if (result.size() != lhs.size()) {
            result.resize(lhs.size());
        }
    }

    Combiner(
        double lhs,
        const VectorList& rhs,
        VectorList& result,
        const std::function<Vector3d (double, const Vector3d&)>& operation
    )
        : mIsScalar(true)
        , mScalar(lhs)
        , mLeft(rhs) // Just make the compiler be quiet.
        , mRight(rhs)
        , mResult(result)
        , mScalarOp(operation)
    {
        if (result.size() != rhs.size()) {
            result.resize(rhs.size());
        }
    }

    void operator ()(const tbb::blocked_range<uint32_t> r) const
    {
        uint32_t i = r.begin();
        uint32_t end = r.end();

        if (mIsScalar) {
            for (; i < end; ++i) {
                mResult[i] = mScalarOp(mScalar, mRight[i]);
            }
        } else {
            for (; i < end; ++i) {
                mResult[i] = mVectorOp(mLeft[i], mRight[i]);
            }
        }
    }

private:
    bool mIsScalar;
    double mScalar;
    const VectorList& mLeft;
    const VectorList& mRight;
    VectorList& mResult;
    std::function<Vector3d (const Vector3d&, const Vector3d&)> mVectorOp;
    std::function<Vector3d (double, const Vector3d&)> mScalarOp;
};

}

// --------------------------------------------------------------------------


VectorList::VectorList() : Collection<Vector3d>()
{
}

VectorList::VectorList(size_t n) : Collection<Vector3d>(n)
{
}

VectorList::VectorList(const VectorList& other) : Collection<Vector3d>(other)
{
}

VectorList::~VectorList()
{
}

void
VectorList::setZero()
{
    QtConcurrent::blockingMap(*this, [](Vector3d& x) { x.setZero(); });
}

// --------------------------------------------------------------------------

VectorList&
operator +=(VectorList& lhs, const VectorList& rhs)
{
    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, lhs.size()),
        Combiner(lhs, rhs, lhs, [=](const Vector3d& x, const Vector3d& y) { return x + y; })
    );
    return lhs;
}

VectorList
operator +(const VectorList& lhs, const VectorList& rhs)
{
    VectorList result;

    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, lhs.size()),
        Combiner(lhs, rhs, result, [=](const Vector3d& x, const Vector3d& y) { return x + y; })
    );

    return result;
}

VectorList&
operator -=(VectorList& lhs, const VectorList& rhs)
{
    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, lhs.size()),
        Combiner(lhs, rhs, lhs, [=](const Vector3d& x, const Vector3d& y) { return x - y; })
    );
    return lhs;
}

VectorList
operator -(const VectorList& lhs, const VectorList& rhs)
{
    VectorList result;

    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, lhs.size()),
        Combiner(lhs, rhs, result, [=](const Vector3d& x, const Vector3d& y) { return x - y; })
    );

    return result;
}

VectorList
operator *(double k, const VectorList& rhs)
{
    VectorList result;

    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, rhs.size()),
        Combiner(k, rhs, result, [=](double x, const Vector3d& y) { return x * y; })
    );

    return result;
}

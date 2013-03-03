#include "SoftBody.h"
#include <iostream>
#include <fstream>
#include <QtCore>
#include <geom/KdTree.h>
#include <geom/Mesh.h>
#include "kernels.h"
#include "../Utils.h" // TODO: dependencies!

using namespace Eigen;

namespace
{

const double EPSILON = 1e-6;
const uint32_t MAX_CG_ITERATIONS = 10;

bool
load(const std::string& filename, VectorList& positions)
{
    std::ifstream file(filename.c_str());
    if (!file.is_open()) {
        return false;
    }

    Vector3d u;
    file >> u[0] >> u[1] >> u[2];
    while (file) {
        positions.push_back(u);
        file >> u[0] >> u[1] >> u[2];
    }

    return true;
}

template <typename T>
void
zero(Collection<T>& list)
{
    QtConcurrent::blockingMap(list, [](T& x) { x.setZero(); });
}

void
zero(std::vector<double>& list)
{
    QtConcurrent::blockingMap(list, [](double& x) { x = 0; });
}

void
identity(SoftBody::MatrixList& matrices)
{
    QtConcurrent::blockingMap(matrices, [](Matrix3d& m) { m.setIdentity(); });
}

double dot(const VectorList& a, const VectorList& b)
{
    assert(a.size() == b.size());

    double result = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        result += a[i](0) * b[i](0)
                + a[i](1) * b[i](1)
                + a[i](2) * b[i](2);
    }
    return result;
}

}

// --------------------------------------------------------------------------

SoftBody::ForceProcessor::ForceProcessor(SoftBody& body, double dt)
    : mBody(body)
    , mDt(dt)
{}

void
SoftBody::ForceProcessor::operator ()(const tbb::blocked_range<uint32_t> r) const
{
    uint32_t lo = r.begin();
    uint32_t hi = r.end();

    mBody.updateRestQuantities(lo, hi);
    mBody.clearForces(lo, hi);
    mBody.computeFs(lo, hi);
    mBody.computeGammas(lo, hi, mDt);
    mBody.decomposeFs(lo, hi);
    mBody.computeStrains(lo, hi);
    mBody.computeStresses(lo, hi);
    mBody.computeForces(lo, hi);
    mBody.applyPlasticDef(lo, hi);
}

SoftBody::MeshProcessor::MeshProcessor(SoftBody& body)
    : mBody(body)
{}

void
SoftBody::MeshProcessor::operator ()(const tbb::blocked_range<uint32_t> r) const
{
    mBody.updateMesh(r.begin(), r.end());
}

// --------------------------------------------------------------------------

SoftBody::SoftBody(const std::string& positionsFile, const Material& material) :
    mBBox(),
    mMaterial(material),
    mMesh(nullptr)
{
    if (!load(positionsFile, posRest)) {
        return; // TODO: Throw exception?
    }

    uint32_t size = posRest.size();

    posWorld.resize(size);
    auto u = posRest.begin();
    for (auto& x : posWorld) {
        x = *u;
        ++u;
    }
    mBBox.add(posWorld);

    bases.resize(size);
    identity(bases);

    velocities.resize(size);
    velocities.setZero();

    accels.resize(size);
    accels.setZero();

    forces.resize(size);

    masses.resize(size);
    volumes.resize(size);
    radii.resize(size);

    neighborhoods.resize(size);

    defs.resize(size);
    elasticDefs.resize(size);
    plasticDefs.resize(size);
    gammas.resize(size);
    alphas.resize(size);
    zero(alphas);

    strains.resize(size);
    zero(strains);

    stresses.resize(size);
    zero(stresses);

    updateNeighborhoods();
    updateRadii();
}

SoftBody::~SoftBody()
{
}

void
SoftBody::setMesh(Mesh* mesh)
{
    Vector3d toTranslate = mBBox.center() - mesh->bbox().center();
    mesh->translate(toTranslate);

    mMesh = mesh;

    auto radius = avgRadius();
    auto kdTree = KdTree(posRest);

    mRestMesh.reserve(mesh->verts().size());
    mMeshNeighborhoods.reserve(mesh->verts().size());

    for (auto& v : mesh->verts()) {
        mRestMesh.push_back(v);

        mMeshNeighborhoods.push_back(Neighborhood());
        auto& hood = mMeshNeighborhoods.back();

        std::vector<uint32_t> indices;
        kdTree.neighbors(posRest, v, -1, radius, indices); // Get ALL neighbors
        for (auto j : indices) {
            auto distance = (posRest[j] - v).norm();
            auto weight = Kernels::standardkernel(radius, distance);
            hood.push_back(Neighbor(j, Vector3d(), weight));
        }
        hood.computeSum();
    }
}

void
SoftBody::updateMesh()
{
    if (mMesh == nullptr) return;

    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, mMesh->verts().size()),
        MeshProcessor(*this)
    );
    //mMesh->updateNormals();
}

void
SoftBody::updateMesh(uint32_t lo, uint32_t hi)
{
    auto rest_it = mRestMesh.begin() + lo;
    auto hood_it = mMeshNeighborhoods.begin() + lo;
    auto vert_it = mMesh->verts().begin() + lo;

    auto end = mMesh->verts().begin() + hi;

    for (; vert_it != end; ++vert_it, ++rest_it, ++hood_it) {
        // compute normalized weighted average of neighbors' displacements.
        Vector3d displacement(0, 0, 0);
        for (auto& n : *hood_it) {
            displacement += n.w * (posWorld[n.index] - posRest[n.index]);
        }
        displacement /= hood_it->sum();

        // set v to rest_it + avg displacement.
        *vert_it = *rest_it + displacement;
    }
}

double
SoftBody::avgRadius() const
{
    double sum = 0;
    for (auto r : radii) {
        sum += r;
    }
    return sum / radii.size();
}

void
SoftBody::clearForces(uint32_t lo, uint32_t hi)
{
    // Clear the forces on each particle.
    QtConcurrent::blockingMap(
        forces.begin() + lo,
        forces.begin() + hi,
        [](Vector3d& f) { f.setZero(); }
    );

    // And clear the temporary forces stored in neighborhoods.
    QtConcurrent::blockingMap(
        neighborhoods.begin() + lo,
        neighborhoods.begin() + hi,
        [](Neighborhood& hood) {
            for (auto& n : hood) {
                n.f.setZero();
            }
        }
    );
}

void
SoftBody::updateState(double dt)
{
    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0, size()),
        ForceProcessor(*this, dt)
    );
    gatherForces();

    // TODO: We only need to embed and update neighorhoods if some neighborhood
    // had some plastic deformation of anything other than Identity.
    embed();
    updateNeighborhoods();
}

void
SoftBody::updateNeighborhoods()
{
    auto kdTree = KdTree(posRest);

    std::vector<uint32_t> indices;
    indices.reserve(Neighborhood::MAX_SIZE + 1);

    auto neighbor_it = neighborhoods.begin();
    auto radius_it = radii.begin();
    uint32_t index = 0;
    for (auto& u : posRest) {
        //
        // A particle's neighborhood should not include itself. However, this
        // kdTree will return an index for the current particle. So increment
        // the neighbor count by 1, and then remove the "self" particle when
        // we're done.
        //

        kdTree.neighbors(posRest, u, Neighborhood::MAX_SIZE + 1, *radius_it, indices);
        auto selfLocation = std::find(indices.begin(), indices.end(), index);
        if (selfLocation != indices.end()) {
            indices.erase(selfLocation);
        }

        // If we find a neighbor we didn't already have, add it with an initial
        // weight of zero.
        //

        Neighborhood newNeighbors;

        for (auto j : indices) {
            if (neighbor_it->hasNeighbor(j)) {
                newNeighbors.push_back(neighbor_it->findNeighbor(j));
            } else {
                Vector3d u_ij = posRest[j] - u;
                Neighbor n(j, u_ij, 0.0);
                newNeighbors.push_back(n);
            }
        }
        *neighbor_it = newNeighbors;

        ++neighbor_it;
        ++radius_it;
        ++index;
    }
}

void
SoftBody::updateRadii()
{
    auto u_it = posRest.begin();
    auto n_it = neighborhoods.begin();
    auto r_it = radii.begin();
    for (; r_it != radii.end(); ++u_it, ++n_it, ++r_it) {
        double avg = 0;
        for (auto& n : *n_it) {
            avg += (*u_it - posRest[n.index]).norm();
        }
        avg /= n_it->size();
        *r_it = avg * 2.0;
    }
}

void
SoftBody::updateRestQuantities(uint32_t lo, uint32_t hi)
{
    // TODO: If there hasn't been any plastic deformation, this can probably be
    // skipped.
    auto x_it = posWorld.begin() + lo;
    auto u_it = posRest.begin() + lo;
    auto n_it = neighborhoods.begin() + lo;
    auto r_it = radii.begin() + lo;
    auto v_it = volumes.begin() + lo;
    auto m_it = masses.begin() + lo;
    auto b_it = bases.begin() + lo;

    auto end = bases.begin() + hi;

    for (; b_it != end; ++x_it, ++u_it, ++n_it, ++r_it, ++v_it, ++m_it, ++b_it) {
        b_it->setZero();
        for (auto& n : *n_it) {
            double distance = (*u_it - posRest[n.index]).norm();
            n.w = Kernels::standardkernel(*r_it, distance);
            n.u = posRest[n.index] - *u_it;
            *b_it += n.u * n.u.transpose() * n.w;
        }
        *v_it = sqrt(b_it->determinant() / Utils::cube(n_it->computeSum()));
        *m_it = mMaterial.density * *v_it;
        *b_it = b_it->inverse().eval();
    }
}

void
SoftBody::computeFs(uint32_t lo, uint32_t hi)
{
    auto n_it = neighborhoods.begin() + lo;
    auto f_it = defs.begin() + lo;
    auto u_it = posWorld.begin() + lo;
    auto b_it = bases.begin() + lo;

    auto end = posWorld.begin() + hi;

    Matrix3d rhs;
    for (; u_it != end; ++u_it, ++f_it, ++n_it, ++b_it) {
        rhs.setZero();
        for (auto& n : *n_it) {
            rhs += n.w * (posWorld[n.index] - *u_it) * n.u.transpose();
        }
        *f_it = Svd(rhs * *b_it, ComputeFullU | ComputeFullV);
    }
}

//
// Uses the previous timestep's stress values to compute gamma.
//
void
SoftBody::computeGammas(uint32_t lo, uint32_t hi, double dt)
{
    auto s_it = stresses.begin() + lo;
    auto g_it = gammas.begin() + lo;
    auto a_it = alphas.begin() + lo;

    auto end = stresses.begin() + hi;

    for (; s_it != end; ++s_it, ++g_it, ++a_it) {
        auto stressNorm = s_it->norm();
        auto& flowRate = mMaterial.flowRate;
        auto& yieldPoint = mMaterial.yieldPoint;

        double gamma = 0;
        if (stressNorm > EPSILON) {
            double k_alpha = mMaterial.hardening * *a_it;
            gamma = flowRate * (stressNorm - yieldPoint - k_alpha) / stressNorm;
        }
        *g_it = Utils::clamp(gamma, 0, 1);
        *a_it += stressNorm * dt;
    }
}

//
// Decomposes the deformation gradient into elastic and plastic portions.
// This method assumes that the gamma values have previously been computed.
//
void
SoftBody::decomposeFs(uint32_t lo, uint32_t hi)
{
    auto f_it = defs.begin() + lo;
    auto p_it = plasticDefs.begin() + lo;
    auto e_it = elasticDefs.begin() + lo;
    auto g_it = gammas.begin() + lo;

    auto end = defs.begin() + hi;

    for (; f_it != end; ++f_it, ++p_it, ++e_it, ++g_it) {
        double det = f_it->singularValues().prod();
        Vector3d fHatStar = f_it->singularValues() / std::fabs(std::cbrt(det));

        *p_it = fHatStar.array().pow(*g_it);
        *e_it = f_it->singularValues().array() / p_it->array();
    }
}

void
SoftBody::computeStrains(uint32_t lo, uint32_t hi)
{
    static const Vector3d I(1, 1, 1);

    auto f_it = elasticDefs.begin() + lo;
    auto e_it = strains.begin() + lo;

    auto end = strains.begin() + hi;

    for (; e_it != end; ++e_it, ++f_it) {
        *e_it = *f_it - I;
    }
}

void
SoftBody::computeStresses(uint32_t lo, uint32_t hi)
{
    auto e_it = strains.begin() + lo;
    auto s_it = stresses.begin() + lo;

    auto end = stresses.begin() + hi;

    for (; s_it != end; ++s_it, ++e_it) {
        double d = mMaterial.lambda * e_it->trace();
        *s_it = (2 * mMaterial.mu * *e_it) + Vector3d(d, d, d);
    }
}

void
SoftBody::computeForces(uint32_t lo, uint32_t hi)
{
    auto def_it = defs.begin() + lo;
    auto vel_it = velocities.begin() + lo;
    auto v_it = volumes.begin() + lo;
    auto stress_it = stresses.begin() + lo;
    auto basis_it = bases.begin() + lo;
    auto n_it = neighborhoods.begin() + lo;
    auto f_it = forces.begin() + lo;

    auto end = forces.begin() + hi;

    for (; f_it != end; ++f_it, ++def_it, ++vel_it, ++v_it, ++stress_it, ++basis_it, ++n_it) {
        double volume = *v_it;
        const Matrix3d& basis = *basis_it;
        const Matrix3d stress = def_it->matrixU()
                              * DiagonalMatrix3d(*stress_it)
                              * def_it->matrixV().transpose();
        Neighborhood& neighborhood = *n_it;

        Matrix3d Fe = -2 * volume * stress * basis;
        for (auto& n : neighborhood) {
            Vector3d force = Fe * (n.u * n.w);
            n.f += force;
            *f_it -= force;

            Vector3d viscous = volume * (velocities[n.index] - *vel_it) * n.w;
            n.f -= viscous;
            *f_it += viscous;
        }
    }
}

void
SoftBody::applyPlasticDef(uint32_t lo, uint32_t hi)
{
    auto f_it = defs.begin() + lo;
    auto p_it = plasticDefs.begin() + lo;
    auto n_it = neighborhoods.begin() + lo;

    auto end = neighborhoods.begin() + hi;

    for (; n_it != end; ++f_it, ++p_it, ++n_it) {
        for (auto& n : *n_it) {
            Matrix3d F = f_it->matrixV()
                       * DiagonalMatrix3d(*p_it)
                       * f_it->matrixV().transpose();
            n.u = F * n.u;
        }
    }
}

void
SoftBody::gatherForces()
{
    for (auto& hood : neighborhoods) {
        for (auto& n : hood) {
            forces[n.index] += n.f;
        }
    }
}

void
SoftBody::embed()
{
    // Ax = b
    // A: K^TK - K is a block matrix consisting of (-w_ij * I_i) and (w_ij * // I_j).
    // x: Solved embedded positions.
    // b: Plastically deformed u vectors: w_ij * u_ij.
    //
    // K:   96N x 3N
    // K^T: 3N x 96N
    // x:   3N
    // b:   96N

    uint32_t m = Neighborhood::MAX_SIZE * size() + 1;

    VectorList x(posRest);
    VectorList b(m);

    // Initialize b
    uint32_t i = 0;
    for (auto& hood : neighborhoods) {
        uint32_t j = 0;
        for (auto& n : hood) {
            b[i + j] = n.w * n.u;
            ++j;
        }
        // Fill in what remains with zeros.
        for (; j < Neighborhood::MAX_SIZE; ++j) {
            b[i + j].setZero();
        }
        i += Neighborhood::MAX_SIZE;
    }
    b[m - 1] = posRest[0];

    cgSolve(x, b);
    for (uint32_t i = 0; i < x.size(); ++i) {
        posRest[i] = x[i];
    }
}

void
SoftBody::applyMatrix(const VectorList& x, VectorList& result)
{
    assert(result.size() == Neighborhood::MAX_SIZE * size());
    assert(x.size() == size());

    result.setZero();

    uint32_t i = 0;
    auto x_it = x.begin();
    for (auto& hood : neighborhoods) {
        uint32_t j = 0;
        for (auto& n : hood) {
            result[i + j] = n.w * (x[n.index] - *x_it);
            ++j;
        }
        i += Neighborhood::MAX_SIZE;
        ++x_it;
    }
    result[result.size()-1] = x[0];
}

VectorList
SoftBody::applyMatrix(const VectorList& x)
{
    assert(x.size() == size());

    VectorList result(Neighborhood::MAX_SIZE * size());
    applyMatrix(x, result);
    return result;
}

void
SoftBody::applyMatrixTranspose(const VectorList& x, VectorList& result)
{
    assert(result.size() == size());
    assert(x.size() == Neighborhood::MAX_SIZE * size());

    result.setZero();

    uint32_t i = 0;
    for (auto& hood : neighborhoods) {
        uint32_t col = i * Neighborhood::MAX_SIZE;
        for (auto& n : hood) {
            auto v = n.w * x[col];
            result[i] -= v;
            result[n.index] += v;
            ++col;
        }
        ++i;
    }
    result[0] += x[x.size()-1];
}

VectorList
SoftBody::applyMatrixTranspose(const VectorList& x)
{
    assert(x.size() == Neighborhood::MAX_SIZE * size());

    VectorList result(size());
    applyMatrixTranspose(x, result);
    return result;
}

void
SoftBody::cgSolve(VectorList& x, const VectorList& b)
{
    assert(x.size() == size());
    assert(b.size() == Neighborhood::MAX_SIZE * size());

    VectorList r = applyMatrixTranspose(b) - applyMatrixTranspose(applyMatrix(x));
    VectorList p(r);
    double rsold = dot(r, r);

    for (uint32_t i = 0; i < MAX_CG_ITERATIONS; ++i) {
        VectorList Ap = applyMatrixTranspose(applyMatrix(p));
        double alpha = rsold / (dot(p, Ap));

        x += alpha * p;
        r -= alpha * Ap;

        double rsnew = dot(r, r);
        if (sqrt(rsnew) < 1e-10) {
            return;
        }

        p = r + (rsnew / rsold) * p;
        rsold = rsnew;
    }
}


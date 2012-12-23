#ifndef PHYSICS_OBSTACLE_H
#define PHYSICS_OBSTACLE_H

#include <Eigen>

class Obstacle
{
public:
    Obstacle();
    Obstacle(double friction);
    virtual ~Obstacle();
    virtual bool bounce(Eigen::Vector3d& x, Eigen::Vector3d& v) const = 0;
    virtual void draw() const {}

    inline double friction() const { return mFriction; }
    inline void setFriction(double f) { mFriction = f; }

protected:
    double mFriction;
};

#endif // PHYSICS_OBSTACLE_H


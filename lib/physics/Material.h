#ifndef PHYSICS_MATERIAL_H
#define PHYSICS_MATERIAL_H

struct Material
{
    double mu;
    double lambda;
    double density;
    double hardening;
    double flowRate;
    double yieldPoint;
};

#endif // PHYSICS_MATERIAL_H

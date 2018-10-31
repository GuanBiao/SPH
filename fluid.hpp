#define _USE_MATH_DEFINES

#ifndef _FLUID_HPP_
#define _FLUID_HPP_

#include <vector>
#include "particle.hpp"

#define REST_DENSITY               998.29f
#define MASS                       0.02f
#define VISCOSITY                  3.5f
#define SURFACE_TENSION            0.0728f
#define THRESHOLD                  7.065f
#define GAS_STIFFNESS              3.0f
#define RESTITUTION                0.5f
#define KERNEL_PARTICLES           20
#define SUPPORT_RADIUS             0.0457f
#define GRAVITATIONAL_ACCELERATION Vector3f(0.0f, -9.82f, 0.0f)

#define TIME_STEP                  0.01f
#define BOX_SIZE                   0.3f

class Fluid {
public:
    Fluid( void );

    void draw( void );
    void simulate( void );

    float calcDensity( Vector3f );
    float calcPressure( float );
    Vector3f calcPressureForce( int, float, float, Vector3f );
    Vector3f calcViscosityForce( int, Vector3f, Vector3f );
    Vector3f calcGravitationalForce( float );
    Vector3f calcSurfaceNormal( Vector3f );
    Vector3f calcSurfaceTensionForce( Vector3f, Vector3f );
    void employEulerIntegrator( Particle &, Vector3f );
    bool detectCollision( Particle, Vector3f &, Vector3f & );
    void updateVelocity( Vector3f &, Vector3f, float );

    float useDefaultKernel( Vector3f, float );
    Vector3f useDefaultKernel_gradient( Vector3f, float );
    float useDefaultKernel_laplacian( Vector3f, float );
    Vector3f usePressureKernel_gradient( Vector3f, float );
    float useViscosityKernel_laplacian( Vector3f, float );

    vector<Particle> mParticles;
};

#endif
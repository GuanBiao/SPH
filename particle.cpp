#include "particle.hpp"

Particle::Particle( float mass, Vector3f position ) {
    mMass                = mass;
    mAcceleration        = Vector3f(0.0f, 0.0f, 0.0f);
    mVelocity            = Vector3f(0.0f, 0.0f, 0.0f);
    mPosition            = position;
    mForce               = Vector3f(0.0f, 0.0f, 0.0f);
    mDensity             = 0.0f;
    mPressure            = 0.0f;
    mPressureForce       = Vector3f(0.0f, 0.0f, 0.0f);
    mViscosityForce      = Vector3f(0.0f, 0.0f, 0.0f);
    mGravitationalForce  = Vector3f(0.0f, 0.0f, 0.0f);
    mSurfaceNormal       = Vector3f(0.0f, 0.0f, 0.0f);
    mSurfaceTensionForce = Vector3f(0.0f, 0.0f, 0.0f);
}
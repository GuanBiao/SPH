#include "fluid.hpp"

const float fluidVolume      = 1000 * MASS / REST_DENSITY;
const float particleDiameter = powf(fluidVolume, 1.0f / 3.0f) / 10;
const float particleRadius   = particleDiameter / 2;

Fluid::Fluid( void ) {
    for (float x = -particleRadius * 9; x <= particleRadius * 9; x += particleDiameter) {
        for (float y = -particleRadius * 9; y <= particleRadius * 9; y += particleDiameter) {
            for (float z = -particleRadius * 9; z <= particleRadius * 9; z += particleDiameter)
                mParticles.push_back(Particle(MASS, Vector3f(x, y, z)));
        }
    }
}

void Fluid::draw( void ) {
    float sphereRadius = powf((3 * MASS) / (4 * M_PI * REST_DENSITY), 1.0f / 3.0f);
    for (int i = 0; i < mParticles.size(); i++) {
        glPushMatrix();
        glTranslatef(mParticles[i].mPosition.x, mParticles[i].mPosition.y, mParticles[i].mPosition.z);
        glutSolidSphere(sphereRadius, 30, 30);
        glPopMatrix();
    }

    glDisable(GL_LIGHTING);

    glColor3f(1.0f, 1.0f, 1.0f);

    // Draw bottom surface edges of the box
    glBegin(GL_LINE_LOOP);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f( BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f( BOX_SIZE / 2, -BOX_SIZE / 2,  BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2,  BOX_SIZE / 2);
    glEnd();

    // Draw top surface edges of the box
    glBegin(GL_LINE_LOOP);
    glVertex3f(-BOX_SIZE / 2,  BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f( BOX_SIZE / 2,  BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f( BOX_SIZE / 2,  BOX_SIZE / 2,  BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2,  BOX_SIZE / 2,  BOX_SIZE / 2);
    glEnd();

    // Draw left surface edges of the box
    glBegin(GL_LINE_LOOP);
    glVertex3f(-BOX_SIZE / 2,  BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2,  BOX_SIZE / 2,  BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2,  BOX_SIZE / 2);
    glVertex3f(-BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glEnd();

    // Draw right surface edges of the box
    glBegin(GL_LINE_LOOP);
    glVertex3f( BOX_SIZE / 2,  BOX_SIZE / 2, -BOX_SIZE / 2);
    glVertex3f( BOX_SIZE / 2,  BOX_SIZE / 2,  BOX_SIZE / 2);
    glVertex3f( BOX_SIZE / 2, -BOX_SIZE / 2,  BOX_SIZE / 2);
    glVertex3f( BOX_SIZE / 2, -BOX_SIZE / 2, -BOX_SIZE / 2);
    glEnd();

    glColor3f(1.0f, 0.0f, 0.0f);

    // Draw x-axis
    glBegin(GL_LINES);
    glVertex3f(-BOX_SIZE, 0.0f, 0.0f);
    glVertex3f( BOX_SIZE, 0.0f, 0.0f);
    glEnd();

    // Draw y-axis
    glBegin(GL_LINES);
    glVertex3f(0.0f, -BOX_SIZE, 0.0f);
    glVertex3f(0.0f,  BOX_SIZE, 0.0f);
    glEnd();

    // Draw z-axis
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, -BOX_SIZE);
    glVertex3f(0.0f, 0.0f,  BOX_SIZE);
    glEnd();

    glEnable(GL_LIGHTING);

}

void Fluid::simulate( void ) {
    // Compute density and pressure
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mDensity  = calcDensity(mParticles[i].mPosition);
        mParticles[i].mPressure = calcPressure(mParticles[i].mDensity);
    }

    // Compute internal forces
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mPressureForce  = calcPressureForce(i, mParticles[i].mDensity, mParticles[i].mPressure, mParticles[i].mPosition);
        mParticles[i].mViscosityForce = calcViscosityForce(i, mParticles[i].mVelocity, mParticles[i].mPosition);
    }

    // Compute external forces
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mGravitationalForce = calcGravitationalForce(mParticles[i].mDensity);
        mParticles[i].mSurfaceNormal      = calcSurfaceNormal(mParticles[i].mPosition);
        if (mParticles[i].mSurfaceNormal.length() >= THRESHOLD)
            mParticles[i].mSurfaceTensionForce = calcSurfaceTensionForce(mParticles[i].mSurfaceNormal, mParticles[i].mPosition);
        else
            mParticles[i].mSurfaceTensionForce = Vector3f(0.0f, 0.0f, 0.0f);
    }

    // Time integration and collision handling
    static float time = 0.0f;
    time += TIME_STEP;
    Vector3f totalForce;
    for (int i = 0; i < mParticles.size(); i++) {
        //totalForce = mParticles[i].mPressureForce + mParticles[i].mViscosityForce + mParticles[i].mSurfaceTensionForce;
        totalForce = mParticles[i].mPressureForce + mParticles[i].mViscosityForce + mParticles[i].mGravitationalForce + mParticles[i].mSurfaceTensionForce;
        employEulerIntegrator(mParticles[i], totalForce);

        Vector3f contactPoint;
        Vector3f unitSurfaceNormal;
        if (detectCollision(mParticles[i], contactPoint, unitSurfaceNormal)) {
            updateVelocity(mParticles[i].mVelocity, unitSurfaceNormal, (mParticles[i].mPosition - contactPoint).length());
            mParticles[i].mPosition = contactPoint;
        }
    }
}

float Fluid::calcDensity( Vector3f position ) {
    float sum = 0.0f;
    for (int j = 0; j < mParticles.size(); j++)
        sum += mParticles[j].mMass * useDefaultKernel(position - mParticles[j].mPosition, SUPPORT_RADIUS);
    return sum;
}

float Fluid::calcPressure( float density ) {
    return GAS_STIFFNESS * (density - REST_DENSITY);
}

Vector3f Fluid::calcPressureForce( int indexOfCurrentParticle, float density, float pressure, Vector3f position ) {
    Vector3f sum(0.0f, 0.0f, 0.0f);
    for (int j = 0; j < mParticles.size(); j++) {
        if (j == indexOfCurrentParticle)
            continue;
        sum += usePressureKernel_gradient(position - mParticles[j].mPosition, SUPPORT_RADIUS) * (pressure / (density * density) + mParticles[j].mPressure / (mParticles[j].mDensity * mParticles[j].mDensity)) * mParticles[j].mMass;
    }
    return -(sum * density);
}

Vector3f Fluid::calcViscosityForce( int indexOfCurrentParticle, Vector3f velocity, Vector3f position ) {
    Vector3f sum(0.0f, 0.0f, 0.0f);
    for (int j = 0; j < mParticles.size(); j++) {
        if (j == indexOfCurrentParticle)
            continue;
        sum += (mParticles[j].mVelocity - velocity) * (mParticles[j].mMass / mParticles[j].mDensity) * useViscosityKernel_laplacian(position - mParticles[j].mPosition, SUPPORT_RADIUS);
    }
    return sum * VISCOSITY;
}

Vector3f Fluid::calcGravitationalForce( float density ) {
    return GRAVITATIONAL_ACCELERATION * density;
}

Vector3f Fluid::calcSurfaceNormal( Vector3f position ) {
    Vector3f sum(0.0f, 0.0f, 0.0f);
    for (int j = 0; j < mParticles.size(); j++)
        sum += useDefaultKernel_gradient(position - mParticles[j].mPosition, SUPPORT_RADIUS) * (mParticles[j].mMass / mParticles[j].mDensity);
    return sum;
}

Vector3f Fluid::calcSurfaceTensionForce( Vector3f surfaceNormal, Vector3f position ) {
    float sum = 0.0f;
    for (int j = 0; j < mParticles.size(); j++)
        sum += (mParticles[j].mMass / mParticles[j].mDensity) * useDefaultKernel_laplacian(position - mParticles[j].mPosition, SUPPORT_RADIUS);
    return -(surfaceNormal.normalize() * SURFACE_TENSION * sum);
}

void Fluid::employEulerIntegrator( Particle &particle, Vector3f totalForce ) {
    particle.mAcceleration = totalForce / particle.mDensity;
    particle.mVelocity     = particle.mVelocity + particle.mAcceleration * TIME_STEP;
    particle.mPosition     = particle.mPosition + particle.mVelocity * TIME_STEP;
}

bool Fluid::detectCollision( Particle particle, Vector3f &contactPoint, Vector3f &unitSurfaceNormal ) {
    if (abs(particle.mPosition.x) <= BOX_SIZE / 2 && abs(particle.mPosition.y) <= BOX_SIZE / 2 && abs(particle.mPosition.z) <= BOX_SIZE / 2)
        return false;

    char maxComponent = 'x';
    float maxValue    = abs(particle.mPosition.x);
    if (maxValue < abs(particle.mPosition.y)) {
        maxComponent = 'y';
        maxValue     = abs(particle.mPosition.y);
    }
    if (maxValue < abs(particle.mPosition.z)) {
        maxComponent = 'z';
        maxValue     = abs(particle.mPosition.z);
    }
    // 'unitSurfaceNormal' is based on the current position component with the largest absolute value
    switch (maxComponent) {
        case 'x':
            if (particle.mPosition.x < -BOX_SIZE / 2) {
                contactPoint = particle.mPosition;            contactPoint.x = -BOX_SIZE / 2;
                if (particle.mPosition.y < -BOX_SIZE / 2)     contactPoint.y = -BOX_SIZE / 2;
                else if (particle.mPosition.y > BOX_SIZE / 2) contactPoint.y =  BOX_SIZE / 2;
                if (particle.mPosition.z < -BOX_SIZE / 2)     contactPoint.z = -BOX_SIZE / 2;
                else if (particle.mPosition.z > BOX_SIZE / 2) contactPoint.z =  BOX_SIZE / 2;
                unitSurfaceNormal = Vector3f( 1.0f,  0.0f,  0.0f);
            }
            else if (particle.mPosition.x > BOX_SIZE / 2) {
                contactPoint = particle.mPosition;            contactPoint.x =  BOX_SIZE / 2;
                if (particle.mPosition.y < -BOX_SIZE / 2)     contactPoint.y = -BOX_SIZE / 2;
                else if (particle.mPosition.y > BOX_SIZE / 2) contactPoint.y =  BOX_SIZE / 2;
                if (particle.mPosition.z < -BOX_SIZE / 2)     contactPoint.z = -BOX_SIZE / 2;
                else if (particle.mPosition.z > BOX_SIZE / 2) contactPoint.z =  BOX_SIZE / 2;
                unitSurfaceNormal = Vector3f(-1.0f,  0.0f,  0.0f);
            }
            break;
        case 'y':
            if (particle.mPosition.y < -BOX_SIZE / 2) {
                contactPoint = particle.mPosition;            contactPoint.y = -BOX_SIZE / 2;
                if (particle.mPosition.x < -BOX_SIZE / 2)     contactPoint.x = -BOX_SIZE / 2;
                else if (particle.mPosition.x > BOX_SIZE / 2) contactPoint.x =  BOX_SIZE / 2;
                if (particle.mPosition.z < -BOX_SIZE / 2)     contactPoint.z = -BOX_SIZE / 2;
                else if (particle.mPosition.z > BOX_SIZE / 2) contactPoint.z =  BOX_SIZE / 2;
                unitSurfaceNormal = Vector3f( 0.0f,  1.0f,  0.0f);
            }
            else if (particle.mPosition.y > BOX_SIZE / 2) {
                contactPoint = particle.mPosition;            contactPoint.y =  BOX_SIZE / 2;
                if (particle.mPosition.x < -BOX_SIZE / 2)     contactPoint.x = -BOX_SIZE / 2;
                else if (particle.mPosition.x > BOX_SIZE / 2) contactPoint.x =  BOX_SIZE / 2;
                if (particle.mPosition.z < -BOX_SIZE / 2)     contactPoint.z = -BOX_SIZE / 2;
                else if (particle.mPosition.z > BOX_SIZE / 2) contactPoint.z =  BOX_SIZE / 2;
                unitSurfaceNormal = Vector3f( 0.0f, -1.0f,  0.0f);
            }
            break;
        case 'z':
            if (particle.mPosition.z < -BOX_SIZE / 2) {
                contactPoint = particle.mPosition;            contactPoint.z = -BOX_SIZE / 2;
                if (particle.mPosition.x < -BOX_SIZE / 2)     contactPoint.x = -BOX_SIZE / 2;
                else if (particle.mPosition.x > BOX_SIZE / 2) contactPoint.x =  BOX_SIZE / 2;
                if (particle.mPosition.y < -BOX_SIZE / 2)     contactPoint.y = -BOX_SIZE / 2;
                else if (particle.mPosition.y > BOX_SIZE / 2) contactPoint.y =  BOX_SIZE / 2;
                unitSurfaceNormal = Vector3f( 0.0f,  0.0f,  1.0f);
            }
            else if (particle.mPosition.z > BOX_SIZE / 2) {
                contactPoint = particle.mPosition;            contactPoint.z =  BOX_SIZE / 2;
                if (particle.mPosition.x < -BOX_SIZE / 2)     contactPoint.x = -BOX_SIZE / 2;
                else if (particle.mPosition.x > BOX_SIZE / 2) contactPoint.x =  BOX_SIZE / 2;
                if (particle.mPosition.y < -BOX_SIZE / 2)     contactPoint.y = -BOX_SIZE / 2;
                else if (particle.mPosition.y > BOX_SIZE / 2) contactPoint.y =  BOX_SIZE / 2;
                unitSurfaceNormal = Vector3f( 0.0f,  0.0f, -1.0f);
            }
            break;
    }
    return true;
}

void Fluid::updateVelocity( Vector3f &velocity, Vector3f unitSurfaceNormal, float penetrationDepth ) {
    velocity = velocity - unitSurfaceNormal * (1 + RESTITUTION * penetrationDepth / (TIME_STEP * velocity.length())) * velocity.dot(unitSurfaceNormal);
}

float Fluid::useDefaultKernel( Vector3f distVector, float supportRadius ) {
    float dist = distVector.length();
    if (dist > supportRadius)
        return 0.0f;
    else
        return (315 / (64 * M_PI * powf(supportRadius, 9.0f))) * powf(supportRadius * supportRadius - dist * dist, 3.0f);
}

Vector3f Fluid::useDefaultKernel_gradient( Vector3f distVector, float supportRadius ) {
    float dist = distVector.length();
    if (dist > supportRadius)
        return Vector3f(0.0f, 0.0f, 0.0f);
    else
        return -(distVector * (945 / (32 * M_PI * powf(supportRadius, 9.0f))) * powf(supportRadius * supportRadius - dist * dist, 2.0f));
}

float Fluid::useDefaultKernel_laplacian( Vector3f distVector, float supportRadius ) {
    float dist = distVector.length();
    if (dist > supportRadius)
        return 0.0f;
    else
        return -(945 / (32 * M_PI * powf(supportRadius, 9.0f))) * (supportRadius * supportRadius - dist * dist) * (3 * supportRadius * supportRadius - 7 * dist * dist);
}

Vector3f Fluid::usePressureKernel_gradient( Vector3f distVector, float supportRadius ) {
    float dist = distVector.length();
    if (dist > supportRadius)
        return Vector3f(0.0f, 0.0f, 0.0f);
    else if (dist < 10e-5) // If ||r|| -> 0+
        return -(Vector3f(1.0f, 1.0f, 1.0f).normalize() * (45 / (M_PI * powf(supportRadius, 6.0f))) * powf(supportRadius - dist, 2.0f));
    else
        return -(distVector.normalize() * (45 / (M_PI * powf(supportRadius, 6.0f))) * powf(supportRadius - dist, 2.0f));
}

float Fluid::useViscosityKernel_laplacian( Vector3f distVector, float supportRadius ) {
    float dist = distVector.length();
    if (dist > supportRadius)
        return 0.0f;
    else
        return (45 / (M_PI * powf(supportRadius, 6.0f))) * (supportRadius - dist);
}
#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <iostream>
#include <cmath>

using namespace std;

struct Vector2i {
    int x;
    int y;

    Vector2i() {}

    Vector2i( int _x, int _y ) {
        x = _x;
        y = _y;
    }
};

struct Vector2f {
    float x;
    float y;

    Vector2f() {}

    Vector2f( float _x, float _y ) {
        x = _x;
        y = _y;
    }
};

struct Vector3i {
    int x;
    int y;
    int z;

    Vector3i() {}

    Vector3i( int _x, int _y, int _z ) {
        x = _x;
        y = _y;
        z = _z;
    }
};

struct Vector3f {
    float x;
    float y;
    float z;

    Vector3f() {}

    Vector3f( float _x, float _y, float _z ) {
        x = _x;
        y = _y;
        z = _z;
    }

    float length( void ) const {
        return sqrtf(x * x + y * y + z * z);
    }

    float dot( const Vector3f &b ) const {
        return x * b.x + y * b.y + z * b.z;
    }

    Vector3f cross( const Vector3f &b ) const {
        Vector3f result(y * b.z - z * b.y,
                        z * b.x - x * b.z,
                        x * b.y - y * b.x);

        return result;
    }

    Vector3f normalize( void ) const {
        float l = length();
        Vector3f result(x / l,
                        y / l,
                        z / l);

        return result;
    }

    Vector3f operator+( const Vector3f &b ) const {
        Vector3f result(x + b.x,
                        y + b.y,
                        z + b.z);

        return result;
    }

    Vector3f operator-( void ) const {
        Vector3f result(-x,
                        -y,
                        -z);

        return result;
    }

    Vector3f operator-( const Vector3f &b ) const {
        Vector3f result(x - b.x,
                        y - b.y,
                        z - b.z);

        return result;
    }

    Vector3f operator*( const float &b ) const {
        Vector3f result(x * b,
                        y * b,
                        z * b);

        return result;
    }

    Vector3f operator/( const float &b ) const {
        Vector3f result(x / b,
                        y / b,
                        z / b);

        return result;
    }

    Vector3f operator+=( const Vector3f &b ) {
        x += b.x;
        y += b.y;
        z += b.z;

        return *this;
    }

    friend ostream &operator<<( ostream &os, const Vector3f &b ) {
        os << "(" << b.x << ", " << b.y << ", " << b.z << ")";

        return os;
    }
};

#endif
#pragma once
#include <cmath>
#include <immintrin.h> 
#include "../ue4math/vector.h"

struct view_matrix_t
{
    float* operator[](int index) {
        return matrix[index];
    }

    float matrix[4][4];
};

struct Vector2 {
    float x, y;
    Vector2() : x(0.0f), y(0.0f) {}
    Vector2(float x_, float y_) : x(x_), y(y_) {}
};

struct Vector3 {
    float x, y, z;

    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    Vector3 operator+(const FVector& v) const {
        return Vector3(x + v.X, y + v.Y, z + v.Z);
    }

    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    Vector3 operator*(float s) const {
        return Vector3(x * s, y * s, z * s);
    }

    float dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    Vector3 operator-() const {
        return Vector3(-x, -y, -z);
    }

    Vector3& operator*=(float s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vector3 normalized() const {
        float length = std::sqrt(x * x + y * y + z * z);
        if (length > 0) {
            return Vector3(x / length, y / length, z / length);
        }
        return *this;
    }

    Vector3 multiply(const Vector3& a) const {
        return Vector3(x * a.x, y * a.y, z * a.z);
    }

    double Length2DSquare() const {
        return std::abs(static_cast<double>(x) * x + static_cast<double>(y) * y);
    }

    bool IsNearlyEqual(const Vector3& other, float tolerance = 0.1f) const {
        return std::abs(x - other.x) <= tolerance &&
            std::abs(y - other.y) <= tolerance &&
            std::abs(z - other.z) <= tolerance;
    }
};

struct Vector4 {
    float x, y, z, w;



    Vector4() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
    Vector4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    Vector4 operator*(const Vector4& q) const {
        return Vector4(
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
            w * q.w - x * q.x - y * q.y - z * q.z
        );
    }

    Vector4 conjugate() const {
        return Vector4(-x, -y, -z, w);
    }

    Vector3 rotate(const Vector3& v) const {
        Vector4 q(v.x, v.y, v.z, 0.0f);
        Vector4 result = (*this) * q * conjugate();
        return Vector3(result.x, result.y, result.z);
    }

    Vector3 rotateInv(const Vector3& v) const {
        Vector4 q(v.x, v.y, v.z, 0.0f);
        Vector4 result = conjugate() * q * (*this);
        return Vector3(result.x, result.y, result.z);
    }

    bool IsNearlyEqual(const Vector4& other, float tolerance = 0.1f) const {
        return std::abs(x - other.x) <= tolerance &&
            std::abs(y - other.y) <= tolerance &&
            std::abs(z - other.z) <= tolerance &&
            std::abs(w - other.w) <= tolerance;
    }
};


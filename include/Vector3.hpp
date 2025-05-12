#pragma once

#include <cmath>
#include <iostream>

namespace PhysicsEngine
{

    class Vector3
    {
    public:
        float x, y, z;

        // Constructors
        Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
        Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
        Vector3(const Vector3 &v) : x(v.x), y(v.y), z(v.z) {}

        // Basic vector operations
        Vector3 operator+(const Vector3 &v) const
        {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        Vector3 operator-(const Vector3 &v) const
        {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        Vector3 operator*(float scalar) const
        {
            return Vector3(x * scalar, y * scalar, z * scalar);
        }

        Vector3 operator/(float scalar) const
        {
            return Vector3(x / scalar, y / scalar, z / scalar);
        }

        // Compound assignment operators
        Vector3 &operator+=(const Vector3 &v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
            return *this;
        }

        Vector3 &operator-=(const Vector3 &v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
            return *this;
        }

        // Vector operations
        float dot(const Vector3 &v) const
        {
            return x * v.x + y * v.y + z * v.z;
        }

        Vector3 cross(const Vector3 &v) const
        {
            return Vector3(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x);
        }

        float length() const
        {
            return std::sqrt(x * x + y * y + z * z);
        }

        float lengthSquared() const
        {
            return x * x + y * y + z * z;
        }

        void normalize()
        {
            float len = length();
            if (len > 0)
            {
                x /= len;
                y /= len;
                z /= len;
            }
        }

        Vector3 normalized() const
        {
            Vector3 result(*this);
            result.normalize();
            return result;
        }

        // Static utility functions
        static float distance(const Vector3 &a, const Vector3 &b)
        {
            return (b - a).length();
        }

        static Vector3 zero()
        {
            return Vector3(0.0f, 0.0f, 0.0f);
        }

        // Stream operator for easy printing
        friend std::ostream &operator<<(std::ostream &os, const Vector3 &v)
        {
            os << "Vector3(" << v.x << ", " << v.y << ", " << v.z << ")";
            return os;
        }
    };

} // namespace PhysicsEngine
#pragma once

#include <Eigen/Core>
#include <iostream>

namespace PhysicsEngine
{
    // Vector3 is now just an alias for Eigen::Vector3f
    using Vector3 = Eigen::Vector3f;

    // Utility functions that might be needed for compatibility
    namespace Vector3Utils
    {
        inline float distance(const Vector3 &a, const Vector3 &b)
        {
            return (b - a).norm();
        }

        inline Vector3 zero()
        {
            return Vector3::Zero();
        }

        inline float length(const Vector3 &v)
        {
            return v.norm();
        }

        inline float lengthSquared(const Vector3 &v)
        {
            return v.squaredNorm();
        }

        inline Vector3 normalize(const Vector3 &v)
        {
            return v.normalized();
        }

        inline float dot(const Vector3 &a, const Vector3 &b)
        {
            return a.dot(b);
        }

        inline Vector3 cross(const Vector3 &a, const Vector3 &b)
        {
            return a.cross(b);
        }
    }
} // namespace PhysicsEngine// Update vector operations
// Enhance vector operations

#pragma once

#include "Collider.hpp"
#include <Eigen/Dense>

namespace PhysicsEngine
{

    class SphereCollider : public Collider
    {
    public:
        explicit SphereCollider(float radius)
            : Collider(Type::Sphere), mRadius(radius)
        {
        }

        // Implement virtual functions
        CollisionInfo checkCollision(const std::shared_ptr<Collider> &other) const override;
        Vector3 getSupport(const Vector3 &direction) const override;
        void updateBounds() override;

        // Sphere-specific methods
        float getRadius() const { return mRadius; }
        void setRadius(float radius) { mRadius = radius; }

    private:
        float mRadius;

        // Helper functions for specific collision types
        CollisionInfo checkSphereCollision(const std::shared_ptr<SphereCollider> &other) const;
    };

} // namespace PhysicsEngine// Add sphere collider improvements
// Improve sphere collider
// Improve sphere collider
// Improve sphere collider precision
// Improve sphere collider performance
// Enhance sphere collider accuracy

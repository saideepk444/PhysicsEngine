#pragma once

#include "Collider.hpp"

namespace PhysicsEngine
{

    class SphereCollider : public Collider
    {
    public:
        SphereCollider(float radius = 1.0f)
            : Collider(Type::Sphere), radius_(radius)
        {
        }

        // Implement virtual functions
        CollisionInfo checkCollision(const std::shared_ptr<Collider> &other) const override;
        Vector3 getSupport(const Vector3 &direction) const override;
        void updateBounds() override;

        // Sphere-specific methods
        float getRadius() const { return radius_; }
        void setRadius(float radius) { radius_ = radius; }

    private:
        float radius_;

        // Helper functions for specific collision types
        CollisionInfo checkSphereCollision(const std::shared_ptr<SphereCollider> &other) const;
    };

} // namespace PhysicsEngine
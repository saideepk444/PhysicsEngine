#pragma once

#include "Vector3.hpp"
#include <memory>

namespace PhysicsEngine
{

    class RigidBody;

    struct CollisionInfo
    {
        Vector3 point;     // Point of collision in world space
        Vector3 normal;    // Normal vector at collision point
        float penetration; // Penetration depth
        bool hasCollision; // Whether a collision occurred
    };

    class Collider
    {
    public:
        enum class Type
        {
            Sphere,
            Box,
            Capsule
        };

        explicit Collider(Type type) : mType(type) {}
        virtual ~Collider() = default;

        // Pure virtual functions for collision detection
        virtual CollisionInfo checkCollision(const std::shared_ptr<Collider> &other) const = 0;
        virtual Vector3 getSupport(const Vector3 &direction) const = 0;
        virtual void updateBounds() = 0;

        // Getters
        Type getType() const { return mType; }
        const Vector3 &getCenter() const { return center; }
        void setCenter(const Vector3 &newCenter) { center = newCenter; }

        // Attach/detach from rigid body
        void attachToRigidBody(std::shared_ptr<RigidBody> body) { rigidBody = body; }
        std::shared_ptr<RigidBody> getRigidBody() const { return rigidBody; }

    protected:
        Type mType;
        Vector3 center;
        std::shared_ptr<RigidBody> rigidBody;
    };

} // namespace PhysicsEngine// Add collision detection improvements
// Update collision detection
// Enhance collision system

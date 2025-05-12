#pragma once

#include "Vector3.hpp"
#include <Eigen/Dense>
#include <memory>

namespace PhysicsEngine
{

    class RigidBody
    {
    public:
        // Physical properties
        float mass;
        float inverseMass;
        Eigen::Matrix3f inertiaTensor;
        Eigen::Matrix3f inverseInertiaTensor;

        // State variables
        Vector3 position;
        Eigen::Quaternionf orientation;
        Vector3 linearVelocity;
        Vector3 angularVelocity;

        // Forces and torques
        Vector3 force;
        Vector3 torque;

        // Constructors
        RigidBody(float mass = 1.0f);

        // Update methods
        void integrate(float dt);
        void applyForce(const Vector3 &force);
        void applyTorque(const Vector3 &torque);
        void applyForceAtPoint(const Vector3 &force, const Vector3 &point);

        // Getters
        Vector3 getPointVelocity(const Vector3 &point) const;
        Eigen::Matrix4f getTransformMatrix() const;

        // Utility methods
        void updateInertiaTensor();
        void clearForces();

    private:
        void updateDerivedData();
    };

} // namespace PhysicsEngine
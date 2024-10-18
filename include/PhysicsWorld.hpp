#pragma once

#include <vector>
#include <memory>
#include "RigidBody.hpp"
#include <Eigen/Dense>

namespace PhysicsEngine
{
    class PhysicsWorld
    {
    public:
        PhysicsWorld(float timeStep = 1.0f / 60.0f);
        ~PhysicsWorld() = default;

        void addRigidBody(std::shared_ptr<RigidBody> body);
        void removeRigidBody(std::shared_ptr<RigidBody> body);
        void step();
        void setGravity(const Eigen::Vector3f &gravity);

        const std::vector<std::shared_ptr<RigidBody>> &getRigidBodies() const;

    private:
        void detectCollisions();

        std::vector<std::shared_ptr<RigidBody>> mBodies;
        float mTimeStep;
        Eigen::Vector3f mGravity;
        int mIterations; // Constraint solver iterations
    };

} // namespace PhysicsEngine// Update physics world interface

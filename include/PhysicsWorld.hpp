#pragma once

#include <vector>
#include <memory>
#include "RigidBody.hpp"
#include "Collider.hpp"
#include <Eigen/Dense>

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
    void resolveCollisions();
    void integrateForces();

    std::vector<std::shared_ptr<RigidBody>> mBodies;
    float mTimeStep;
    Eigen::Vector3f mGravity;
    const int mIterations = 10; // Constraint solver iterations
};
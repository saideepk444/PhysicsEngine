#include "PhysicsWorld.hpp"
#include <algorithm>

PhysicsWorld::PhysicsWorld(float timeStep)
    : mTimeStep(timeStep), mGravity(0.0f, -9.81f, 0.0f) // Default gravity
{
}

void PhysicsWorld::addRigidBody(std::shared_ptr<RigidBody> body)
{
    mBodies.push_back(body);
}

void PhysicsWorld::removeRigidBody(std::shared_ptr<RigidBody> body)
{
    mBodies.erase(
        std::remove(mBodies.begin(), mBodies.end(), body),
        mBodies.end());
}

void PhysicsWorld::step()
{
    detectCollisions();

    // Solve constraints multiple times for stability
    for (int i = 0; i < mIterations; ++i)
    {
        resolveCollisions();
    }

    integrateForces();
}

void PhysicsWorld::setGravity(const Eigen::Vector3f &gravity)
{
    mGravity = gravity;
}

const std::vector<std::shared_ptr<RigidBody>> &PhysicsWorld::getRigidBodies() const
{
    return mBodies;
}

void PhysicsWorld::detectCollisions()
{
    // Broad phase - using simple N^2 for now
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        for (size_t j = i + 1; j < mBodies.size(); ++j)
        {
            auto &bodyA = mBodies[i];
            auto &bodyB = mBodies[j];

            // Skip if both bodies are static
            if (!bodyA->isDynamic() && !bodyB->isDynamic())
            {
                continue;
            }

            // Check for collision between colliders
            if (bodyA->getCollider() && bodyB->getCollider())
            {
                // TODO: Implement collision detection between different collider types
                // For now, assuming sphere colliders
                // Will be expanded in subsequent implementations
            }
        }
    }
}

void PhysicsWorld::resolveCollisions()
{
    for (auto &body : mBodies)
    {
        if (!body->isDynamic())
            continue;

        // Apply collision impulses
        // TODO: Implement proper collision response
        // This will be expanded with proper impulse calculations
    }
}

void PhysicsWorld::integrateForces()
{
    for (auto &body : mBodies)
    {
        if (!body->isDynamic())
            continue;

        // Semi-implicit Euler integration
        Eigen::Vector3f acceleration = mGravity + body->getForce() * body->getInverseMass();
        body->setVelocity(body->getVelocity() + acceleration * mTimeStep);
        body->setPosition(body->getPosition() + body->getVelocity() * mTimeStep);

        // Angular motion
        Eigen::Vector3f angularAccel = body->getInverseInertia() * body->getTorque();
        body->setAngularVelocity(body->getAngularVelocity() + angularAccel * mTimeStep);

        // Reset forces for next frame
        body->clearForces();
    }
}
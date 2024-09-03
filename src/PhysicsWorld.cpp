#include "PhysicsWorld.hpp"
#include <algorithm>

namespace PhysicsEngine
{

    PhysicsWorld::PhysicsWorld(float timeStep)
        : mTimeStep(timeStep), mGravity(0.0f, -9.81f, 0.0f), mIterations(10) // Default values
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
        // Apply forces (including gravity)
        for (auto &body : mBodies)
        {
            if (!body->isDynamic())
                continue;

            // Apply gravity
            body->applyForce(mGravity * body->getMass());
        }

        // Simple collision detection and response
        detectCollisions();

        // Integrate motion
        for (auto &body : mBodies)
        {
            if (!body->isDynamic())
                continue;

            // Update velocities
            Eigen::Vector3f acceleration = body->getForce() * body->getInverseMass();
            body->setVelocity(body->getVelocity() + acceleration * mTimeStep);

            // Update positions
            body->setPosition(body->getPosition() + body->getVelocity() * mTimeStep);

            // Update angular motion
            Eigen::Vector3f angularAccel = body->getInverseInertia() * body->getTorque();
            body->setAngularVelocity(body->getAngularVelocity() + angularAccel * mTimeStep);

            // Clear forces for next frame
            body->clearForces();
        }
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
        // Simple sphere collision detection
        for (size_t i = 0; i < mBodies.size(); ++i)
        {
            for (size_t j = i + 1; j < mBodies.size(); ++j)
            {
                auto &bodyA = mBodies[i];
                auto &bodyB = mBodies[j];

                // Skip if both bodies are static
                if (!bodyA->isDynamic() && !bodyB->isDynamic())
                    continue;

                // Simple sphere collision check
                Eigen::Vector3f delta = bodyA->getPosition() - bodyB->getPosition();
                float distance = delta.norm();
                float minDist = 2.0f; // Assuming sphere radius of 1.0

                if (distance < minDist)
                {
                    // Collision response
                    Eigen::Vector3f normal = delta.normalized();
                    float overlap = minDist - distance;

                    // Separate the objects
                    if (bodyA->isDynamic())
                    {
                        bodyA->setPosition(bodyA->getPosition() + normal * overlap * 0.5f);
                    }
                    if (bodyB->isDynamic())
                    {
                        bodyB->setPosition(bodyB->getPosition() - normal * overlap * 0.5f);
                    }

                    // Simple elastic collision
                    if (bodyA->isDynamic() && bodyB->isDynamic())
                    {
                        Eigen::Vector3f relativeVel = bodyA->getVelocity() - bodyB->getVelocity();
                        float restitution = 0.8f;
                        float j = -(1.0f + restitution) * relativeVel.dot(normal) /
                                  (bodyA->getInverseMass() + bodyB->getInverseMass());

                        bodyA->setVelocity(bodyA->getVelocity() + normal * j * bodyA->getInverseMass());
                        bodyB->setVelocity(bodyB->getVelocity() - normal * j * bodyB->getInverseMass());
                    }
                    else if (bodyA->isDynamic())
                    {
                        // Collision with static object
                        Eigen::Vector3f velAlongNormal = normal * bodyA->getVelocity().dot(normal);
                        bodyA->setVelocity(bodyA->getVelocity() - velAlongNormal * 1.8f);
                    }
                    else if (bodyB->isDynamic())
                    {
                        // Collision with static object
                        Eigen::Vector3f velAlongNormal = normal * bodyB->getVelocity().dot(normal);
                        bodyB->setVelocity(bodyB->getVelocity() + velAlongNormal * 1.8f);
                    }
                }
            }
        }
    }
}// Add error handling improvements
// Fix minor bugs in physics world

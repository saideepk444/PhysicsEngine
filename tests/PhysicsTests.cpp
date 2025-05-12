#include <gtest/gtest.h>
#include "PhysicsWorld.hpp"
#include "RigidBody.hpp"
#include "SphereCollider.hpp"

TEST(PhysicsTests, GravityTest)
{
    PhysicsWorld world(1.0f / 60.0f);
    world.setGravity(Eigen::Vector3f(0.0f, -9.81f, 0.0f));

    auto body = std::make_shared<RigidBody>();
    body->setPosition(Eigen::Vector3f(0.0f, 10.0f, 0.0f));
    body->setMass(1.0f);
    world.addRigidBody(body);

    // Simulate for 1 second
    for (int i = 0; i < 60; ++i)
    {
        world.step();
    }

    // Check that object has fallen due to gravity
    EXPECT_LT(body->getPosition().y(), 10.0f);
}

TEST(PhysicsTests, CollisionTest)
{
    PhysicsWorld world(1.0f / 60.0f);
    world.setGravity(Eigen::Vector3f(0.0f, -9.81f, 0.0f));

    // Create two spheres that should collide
    auto sphere1 = std::make_shared<RigidBody>();
    sphere1->setPosition(Eigen::Vector3f(0.0f, 5.0f, 0.0f));
    sphere1->setMass(1.0f);
    auto collider1 = std::make_shared<SphereCollider>(1.0f);
    sphere1->setCollider(collider1);
    world.addRigidBody(sphere1);

    auto sphere2 = std::make_shared<RigidBody>();
    sphere2->setPosition(Eigen::Vector3f(0.0f, 2.0f, 0.0f));
    sphere2->setMass(1.0f);
    auto collider2 = std::make_shared<SphereCollider>(1.0f);
    sphere2->setCollider(collider2);
    world.addRigidBody(sphere2);

    // Initial distance between spheres
    float initialDistance = (sphere1->getPosition() - sphere2->getPosition()).norm();

    // Simulate for a short time
    for (int i = 0; i < 30; ++i)
    {
        world.step();
    }

    // Check that spheres have moved closer together
    float finalDistance = (sphere1->getPosition() - sphere2->getPosition()).norm();
    EXPECT_LT(finalDistance, initialDistance);
}

TEST(PhysicsTests, ConservationOfMomentum)
{
    PhysicsWorld world(1.0f / 60.0f);
    world.setGravity(Eigen::Vector3f(0.0f, 0.0f, 0.0f)); // No gravity for this test

    // Create two bodies for elastic collision
    auto body1 = std::make_shared<RigidBody>();
    body1->setPosition(Eigen::Vector3f(-1.0f, 0.0f, 0.0f));
    body1->setVelocity(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    body1->setMass(1.0f);
    world.addRigidBody(body1);

    auto body2 = std::make_shared<RigidBody>();
    body2->setPosition(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    body2->setVelocity(Eigen::Vector3f(-1.0f, 0.0f, 0.0f));
    body2->setMass(1.0f);
    world.addRigidBody(body2);

    // Calculate initial momentum
    Eigen::Vector3f initialMomentum =
        body1->getMass() * body1->getVelocity() +
        body2->getMass() * body2->getVelocity();

    // Simulate collision
    for (int i = 0; i < 60; ++i)
    {
        world.step();
    }

    // Calculate final momentum
    Eigen::Vector3f finalMomentum =
        body1->getMass() * body1->getVelocity() +
        body2->getMass() * body2->getVelocity();

    // Check conservation of momentum (with some tolerance for numerical errors)
    EXPECT_NEAR(initialMomentum.norm(), finalMomentum.norm(), 1e-5);
}
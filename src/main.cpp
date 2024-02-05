#include "PhysicsWorld.hpp"
#include "Renderer.hpp"
#include "RigidBody.hpp"
#include "SphereCollider.hpp"
#include <memory>
#include <chrono>
#include <thread>

using namespace PhysicsEngine;

int main()
{
    // Initialize physics world
    PhysicsWorld world(1.0f / 60.0f);
    world.setGravity(Eigen::Vector3f(0.0f, -9.81f, 0.0f));

    // Initialize renderer
    Renderer renderer(1024, 768);
    if (!renderer.initialize())
    {
        return -1;
    }

    // Create some test objects
    // Ground plane (static)
    auto ground = std::make_shared<RigidBody>();
    ground->setPosition(Eigen::Vector3f(0.0f, -5.0f, 0.0f));
    ground->setDynamic(false);
    world.addRigidBody(ground);

    // Falling sphere
    auto sphere = std::make_shared<RigidBody>();
    sphere->setPosition(Eigen::Vector3f(0.0f, 5.0f, 0.0f));
    sphere->setMass(1.0f);
    auto sphereCollider = std::make_shared<SphereCollider>(1.0f);
    sphere->setCollider(sphereCollider);
    world.addRigidBody(sphere);

    // Main loop
    auto lastTime = std::chrono::high_resolution_clock::now();
    const auto fixedTimeStep = std::chrono::duration<float>(1.0f / 60.0f);

    while (!renderer.shouldClose())
    {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto deltaTime = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - lastTime);
        lastTime = currentTime;

        // Physics update
        world.step();

        // Render
        renderer.clear();
        renderer.render(world);
        renderer.swapBuffers();

        // Cap frame rate
        auto frameTime = std::chrono::high_resolution_clock::now() - currentTime;
        if (frameTime < fixedTimeStep)
        {
            std::this_thread::sleep_for(fixedTimeStep - frameTime);
        }
    }

    return 0;
}// Minor optimization update
// Add performance optimizations
// Add code comments
// Add performance improvements
// Add memory optimization
// Add performance monitoring

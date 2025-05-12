#include "RigidBody.hpp"

namespace PhysicsEngine
{

    RigidBody::RigidBody(float mass_) : mass(mass_)
    {
        if (mass <= 0)
        {
            mass = 1.0f;
        }
        inverseMass = 1.0f / mass;

        // Initialize state variables
        position = Vector3::zero();
        orientation = Eigen::Quaternionf::Identity();
        linearVelocity = Vector3::zero();
        angularVelocity = Vector3::zero();

        // Initialize forces and torques
        force = Vector3::zero();
        torque = Vector3::zero();

        // Initialize inertia tensor (assuming cube shape initially)
        float i = mass * 1.0f / 6.0f; // For a 1x1x1 cube
        inertiaTensor = Eigen::Matrix3f::Identity() * i;
        inverseInertiaTensor = inertiaTensor.inverse();
    }

    void RigidBody::integrate(float dt)
    {
        // Update linear motion
        Vector3 acceleration = force * inverseMass;
        linearVelocity += acceleration * dt;
        position += linearVelocity * dt;

        // Update angular motion
        Vector3 angularAcceleration(
            inverseInertiaTensor * Eigen::Vector3f(torque.x, torque.y, torque.z));
        angularVelocity += Vector3(
                               angularAcceleration.x,
                               angularAcceleration.y,
                               angularAcceleration.z) *
                           dt;

        // Update orientation
        Eigen::Quaternionf spin(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
        Eigen::Quaternionf orientationChange = spin * orientation * 0.5f * dt;
        orientation.coeffs() += orientationChange.coeffs();
        orientation.normalize();

        // Clear forces for next frame
        clearForces();
        updateDerivedData();
    }

    void RigidBody::applyForce(const Vector3 &f)
    {
        force += f;
    }

    void RigidBody::applyTorque(const Vector3 &t)
    {
        torque += t;
    }

    void RigidBody::applyForceAtPoint(const Vector3 &f, const Vector3 &point)
    {
        // Apply force
        force += f;

        // Calculate torque: τ = r × F
        Vector3 r = point - position;
        torque += r.cross(f);
    }

    Vector3 RigidBody::getPointVelocity(const Vector3 &point) const
    {
        Vector3 r = point - position;
        return linearVelocity + angularVelocity.cross(r);
    }

    Eigen::Matrix4f RigidBody::getTransformMatrix() const
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // Set rotation part
        Eigen::Matrix3f rot = orientation.toRotationMatrix();
        transform.block<3, 3>(0, 0) = rot;

        // Set translation part
        transform(0, 3) = position.x;
        transform(1, 3) = position.y;
        transform(2, 3) = position.z;

        return transform;
    }

    void RigidBody::updateInertiaTensor()
    {
        // This is a simplified version - in a real engine, you'd update this based on the
        // actual shape of the rigid body
        float i = mass * 1.0f / 6.0f;
        inertiaTensor = Eigen::Matrix3f::Identity() * i;
        inverseInertiaTensor = inertiaTensor.inverse();
    }

    void RigidBody::clearForces()
    {
        force = Vector3::zero();
        torque = Vector3::zero();
    }

    void RigidBody::updateDerivedData()
    {
        // Update any derived quantities that depend on position/orientation
        // This is a placeholder for more complex implementations
    }

} // namespace PhysicsEngine
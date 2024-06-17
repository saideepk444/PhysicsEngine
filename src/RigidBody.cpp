#include "RigidBody.hpp"

namespace PhysicsEngine
{
    RigidBody::RigidBody()
        : mPosition(Eigen::Vector3f::Zero()),
          mVelocity(Eigen::Vector3f::Zero()),
          mForce(Eigen::Vector3f::Zero()),
          mTorque(Eigen::Vector3f::Zero()),
          mAngularVelocity(Eigen::Vector3f::Zero()),
          mOrientation(Eigen::Quaternionf::Identity()),
          mMass(1.0f),
          mInverseMass(1.0f),
          mInverseInertia(Eigen::Matrix3f::Identity()),
          mIsDynamic(true),
          mCollider(nullptr)
    {
    }

    void RigidBody::setMass(float mass)
    {
        if (mass <= 0)
        {
            mMass = 1.0f;
            mInverseMass = 1.0f;
            return;
        }

        mMass = mass;
        mInverseMass = 1.0f / mass;

        // Update inertia tensor (assuming sphere for simplicity)
        float i = (2.0f / 5.0f) * mMass; // For a solid sphere
        mInverseInertia = Eigen::Matrix3f::Identity() * (1.0f / i);
    }

    void RigidBody::integrate(float dt)
    {
        if (!mIsDynamic)
            return;

        // Update linear motion
        Eigen::Vector3f acceleration = mForce * mInverseMass;
        mVelocity += acceleration * dt;
        mPosition += mVelocity * dt;

        // Update angular motion
        Eigen::Vector3f angularAcceleration = mInverseInertia * mTorque;
        mAngularVelocity += angularAcceleration * dt;

        // Update orientation using quaternion differential equation
        Eigen::Quaternionf omega(0, mAngularVelocity.x(), mAngularVelocity.y(), mAngularVelocity.z());
        Eigen::Quaternionf orientationDeriv = omega * mOrientation;
        mOrientation.coeffs() += orientationDeriv.coeffs() * (dt * 0.5f);
        mOrientation.normalize();

        // Clear forces for next frame
        clearForces();
        updateDerivedData();
    }

    void RigidBody::applyForceAtPoint(const Eigen::Vector3f &force, const Eigen::Vector3f &point)
    {
        // Apply force
        applyForce(force);

        // Calculate torque: τ = r × F
        Eigen::Vector3f r = point - mPosition;
        applyTorque(r.cross(force));
    }

    Eigen::Vector3f RigidBody::getPointVelocity(const Eigen::Vector3f &point) const
    {
        Eigen::Vector3f r = point - mPosition;
        return mVelocity + mAngularVelocity.cross(r);
    }

    Eigen::Matrix4f RigidBody::getTransformMatrix() const
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // Set rotation part
        Eigen::Matrix3f rot = mOrientation.toRotationMatrix();
        transform.block<3, 3>(0, 0) = rot;

        // Set translation part
        transform.block<3, 1>(0, 3) = mPosition;

        return transform;
    }

    void RigidBody::updateDerivedData()
    {
        // Update any derived quantities that depend on position/orientation
        if (mCollider)
        {
            mCollider->setCenter(mPosition);
            mCollider->updateBounds();
        }
    }

} // namespace PhysicsEngine// Fix memory management
// Add better error handling
// Update rigid body physics

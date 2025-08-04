#pragma once

#include <Eigen/Dense>
#include <memory>
#include "Collider.hpp"

namespace PhysicsEngine
{

    class RigidBody
    {
    public:
        RigidBody();
        ~RigidBody() = default;

        // Getters
        const Eigen::Vector3f &getPosition() const { return mPosition; }
        const Eigen::Vector3f &getVelocity() const { return mVelocity; }
        const Eigen::Vector3f &getForce() const { return mForce; }
        const Eigen::Vector3f &getTorque() const { return mTorque; }
        const Eigen::Vector3f &getAngularVelocity() const { return mAngularVelocity; }
        float getMass() const { return mMass; }
        float getInverseMass() const { return mInverseMass; }
        const Eigen::Matrix3f &getInverseInertia() const { return mInverseInertia; }
        bool isDynamic() const { return mIsDynamic; }
        std::shared_ptr<Collider> getCollider() const { return mCollider; }
        Eigen::Matrix4f getTransformMatrix() const;

        // Setters
        void setPosition(const Eigen::Vector3f &position) { mPosition = position; }
        void setVelocity(const Eigen::Vector3f &velocity) { mVelocity = velocity; }
        void setAngularVelocity(const Eigen::Vector3f &angularVelocity) { mAngularVelocity = angularVelocity; }
        void setMass(float mass);
        void setDynamic(bool isDynamic) { mIsDynamic = isDynamic; }
        void setCollider(std::shared_ptr<Collider> collider) { mCollider = collider; }

        // Physics methods
        void integrate(float dt);
        void applyForce(const Eigen::Vector3f &force) { mForce += force; }
        void applyTorque(const Eigen::Vector3f &torque) { mTorque += torque; }
        void applyForceAtPoint(const Eigen::Vector3f &force, const Eigen::Vector3f &point);
        Eigen::Vector3f getPointVelocity(const Eigen::Vector3f &point) const;
        void clearForces()
        {
            mForce.setZero();
            mTorque.setZero();
        }

    private:
        void updateDerivedData();

        Eigen::Vector3f mPosition{Eigen::Vector3f::Zero()};
        Eigen::Vector3f mVelocity{Eigen::Vector3f::Zero()};
        Eigen::Vector3f mForce{Eigen::Vector3f::Zero()};
        Eigen::Vector3f mTorque{Eigen::Vector3f::Zero()};
        Eigen::Vector3f mAngularVelocity{Eigen::Vector3f::Zero()};
        Eigen::Quaternionf mOrientation{Eigen::Quaternionf::Identity()};

        float mMass{1.0f};
        float mInverseMass{1.0f};
        Eigen::Matrix3f mInverseInertia{Eigen::Matrix3f::Identity()};

        bool mIsDynamic{true};
        std::shared_ptr<Collider> mCollider;
    };

} // namespace PhysicsEngine
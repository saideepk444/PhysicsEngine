#pragma once

#include <GLFW/glfw3.h>
#include <memory>
#include "PhysicsWorld.hpp"
#include <Eigen/Dense>

class Renderer
{
public:
    Renderer(int width = 1024, int height = 768);
    ~Renderer();

    bool initialize();
    void render(const PhysicsWorld &world);
    bool shouldClose() const;
    void clear();
    void swapBuffers();

private:
    void drawSphere(const Eigen::Vector3f &position, float radius);
    void drawBox(const Eigen::Vector3f &position, const Eigen::Vector3f &dimensions);
    void setupLighting();
    void setupCamera();

    GLFWwindow *mWindow;
    int mWidth;
    int mHeight;

    // Camera parameters
    Eigen::Vector3f mCameraPos;
    Eigen::Vector3f mCameraTarget;
    float mCameraFOV;
};
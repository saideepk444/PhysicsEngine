#include "Renderer.hpp"
#include <OpenGL/gl.h>
#include <cmath>

namespace PhysicsEngine
{

    Renderer::Renderer(int width, int height)
        : mWidth(width), mHeight(height), mWindow(nullptr), mCameraPos(0.0f, 5.0f, 10.0f), mCameraTarget(0.0f, 0.0f, 0.0f), mCameraFOV(45.0f)
    {
    }

    Renderer::~Renderer()
    {
        if (mWindow)
        {
            glfwDestroyWindow(mWindow);
        }
        glfwTerminate();
    }

    bool Renderer::initialize()
    {
        if (!glfwInit())
        {
            return false;
        }

        mWindow = glfwCreateWindow(mWidth, mHeight, "Physics Engine", nullptr, nullptr);
        if (!mWindow)
        {
            glfwTerminate();
            return false;
        }

        glfwMakeContextCurrent(mWindow);

        // Setup OpenGL state
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);

        return true;
    }

    void Renderer::setupLighting()
    {
        GLfloat light_position[] = {10.0f, 10.0f, 10.0f, 1.0f};
        GLfloat light_ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
        GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
        GLfloat light_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};

        glLightfv(GL_LIGHT0, GL_POSITION, light_position);
        glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
        glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    }

    void Renderer::setupCamera()
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(mCameraFOV, (GLfloat)mWidth / (GLfloat)mHeight, 0.1f, 100.0f);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(mCameraPos.x(), mCameraPos.y(), mCameraPos.z(),
                  mCameraTarget.x(), mCameraTarget.y(), mCameraTarget.z(),
                  0.0f, 1.0f, 0.0f);
    }

    void Renderer::drawSphere(const Eigen::Vector3f &position, float radius)
    {
        glPushMatrix();
        glTranslatef(position.x(), position.y(), position.z());
        GLUquadric *quad = gluNewQuadric();
        gluSphere(quad, radius, 32, 32);
        gluDeleteQuadric(quad);
        glPopMatrix();
    }

    void Renderer::drawBox(const Eigen::Vector3f &position, const Eigen::Vector3f &dimensions)
    {
        glPushMatrix();
        glTranslatef(position.x(), position.y(), position.z());

        glBegin(GL_QUADS);
        // Front face
        glNormal3f(0.0f, 0.0f, 1.0f);
        glVertex3f(-dimensions.x(), -dimensions.y(), dimensions.z());
        glVertex3f(dimensions.x(), -dimensions.y(), dimensions.z());
        glVertex3f(dimensions.x(), dimensions.y(), dimensions.z());
        glVertex3f(-dimensions.x(), dimensions.y(), dimensions.z());

        // Back face
        glNormal3f(0.0f, 0.0f, -1.0f);
        glVertex3f(-dimensions.x(), -dimensions.y(), -dimensions.z());
        glVertex3f(-dimensions.x(), dimensions.y(), -dimensions.z());
        glVertex3f(dimensions.x(), dimensions.y(), -dimensions.z());
        glVertex3f(dimensions.x(), -dimensions.y(), -dimensions.z());

        // Top face
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(-dimensions.x(), dimensions.y(), -dimensions.z());
        glVertex3f(-dimensions.x(), dimensions.y(), dimensions.z());
        glVertex3f(dimensions.x(), dimensions.y(), dimensions.z());
        glVertex3f(dimensions.x(), dimensions.y(), -dimensions.z());

        // Bottom face
        glNormal3f(0.0f, -1.0f, 0.0f);
        glVertex3f(-dimensions.x(), -dimensions.y(), -dimensions.z());
        glVertex3f(dimensions.x(), -dimensions.y(), -dimensions.z());
        glVertex3f(dimensions.x(), -dimensions.y(), dimensions.z());
        glVertex3f(-dimensions.x(), -dimensions.y(), dimensions.z());

        // Right face
        glNormal3f(1.0f, 0.0f, 0.0f);
        glVertex3f(dimensions.x(), -dimensions.y(), -dimensions.z());
        glVertex3f(dimensions.x(), dimensions.y(), -dimensions.z());
        glVertex3f(dimensions.x(), dimensions.y(), dimensions.z());
        glVertex3f(dimensions.x(), -dimensions.y(), dimensions.z());

        // Left face
        glNormal3f(-1.0f, 0.0f, 0.0f);
        glVertex3f(-dimensions.x(), -dimensions.y(), -dimensions.z());
        glVertex3f(-dimensions.x(), -dimensions.y(), dimensions.z());
        glVertex3f(-dimensions.x(), dimensions.y(), dimensions.z());
        glVertex3f(-dimensions.x(), dimensions.y(), -dimensions.z());
        glEnd();

        glPopMatrix();
    }

    void Renderer::render(const PhysicsWorld &world)
    {
        setupCamera();
        setupLighting();

        // Draw ground plane
        glColor3f(0.8f, 0.8f, 0.8f);
        glPushMatrix();
        glTranslatef(0.0f, -5.0f, 0.0f);
        glScalef(10.0f, 0.1f, 10.0f);
        drawBox(Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones());
        glPopMatrix();

        for (const auto &body : world.getRigidBodies())
        {
            // Set color based on object type
            if (body->isDynamic())
            {
                glColor3f(0.2f, 0.6f, 0.8f);
            }
            else
            {
                glColor3f(0.8f, 0.8f, 0.8f);
            }

            // Draw based on collider type
            drawSphere(body->getPosition(), 1.0f);
        }
    }

    bool Renderer::shouldClose() const
    {
        return glfwWindowShouldClose(mWindow);
    }

    void Renderer::clear()
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void Renderer::swapBuffers()
    {
        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }

} // namespace PhysicsEngine// Performance optimization
// Improve rendering quality

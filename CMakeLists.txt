cmake_minimum_required(VERSION 3.15)
project(PhysicsEngine VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GLU REQUIRED)

# Add source files
set(SOURCES
    src/RigidBody.cpp
    src/PhysicsWorld.cpp
    src/Renderer.cpp
    src/main.cpp
)

# Add header files
set(HEADERS
    include/RigidBody.hpp
    include/PhysicsWorld.hpp
    include/Renderer.hpp
    include/Collider.hpp
    include/SphereCollider.hpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${OPENGL_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    OpenGL::GL
    glfw
    Eigen3::Eigen
    ${GLU_LIBRARIES}
) 
add_library(PhysicsCore
    RigidBody.cpp
    PhysicsWorld.cpp
    Renderer.cpp
)

target_include_directories(PhysicsCore
    PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(PhysicsCore
    PUBLIC
    OpenGL::GL
    glfw
    Eigen3::Eigen
)

# Main executable
add_executable(PhysicsDemo main.cpp)
target_link_libraries(PhysicsDemo 
    PRIVATE 
    PhysicsCore
    ${OPENGL_LIBRARIES}
    glfw
) 
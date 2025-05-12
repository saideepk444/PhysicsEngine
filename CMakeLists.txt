cmake_minimum_required(VERSION 3.15)
project(PhysicsEngine VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(tests)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OPENGL_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Main executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    OpenGL::GL
    glfw
    Eigen3::Eigen
) 
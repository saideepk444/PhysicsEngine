# Physics Engine

A high-performance physics simulation engine implemented in C++ for modeling rigid body dynamics, collision detection, and particle systems. The engine provides real-time visualization using OpenGL and supports various physical simulations.

## Features

- Rigid body dynamics simulation
- Collision detection and response
- Real-time 3D visualization using OpenGL
- Support for different collider types (spheres, boxes)
- Gravity and force application
- Optimized integration methods for stable simulations

## Dependencies

- CMake (>= 3.15)
- C++17 compatible compiler
- OpenGL
- GLFW3
- Eigen3

## Building the Project

```bash
# Create build directory
mkdir build && cd build

# Configure CMake
cmake ..

# Build the project
cmake --build .
```

## Running the Demo

After building, you can run the physics demo from the build directory:

```bash
./src/PhysicsDemo
```

## Project Structure

- `include/` - Header files
  - `PhysicsWorld.hpp` - Main physics simulation manager
  - `RigidBody.hpp` - Rigid body implementation
  - `Collider.hpp` - Base collider interface
  - `SphereCollider.hpp` - Sphere collider implementation
  - `Renderer.hpp` - OpenGL visualization system

- `src/` - Source files
  - `main.cpp` - Demo application
  - `PhysicsWorld.cpp` - Physics simulation implementation
  - `RigidBody.cpp` - Rigid body dynamics implementation
  - `Renderer.cpp` - Visualization system implementation

## Usage Example

```cpp
// Initialize physics world
PhysicsWorld world(1.0f/60.0f);
world.setGravity(Eigen::Vector3f(0.0f, -9.81f, 0.0f));

// Create a dynamic sphere
auto sphere = std::make_shared<RigidBody>();
sphere->setPosition(Eigen::Vector3f(0.0f, 5.0f, 0.0f));
sphere->setMass(1.0f);
auto sphereCollider = std::make_shared<SphereCollider>(1.0f);
sphere->setCollider(sphereCollider);
world.addRigidBody(sphere);

// Simulate physics
world.step();
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details. <-v Updated documentation -->
<-v Update project documentation -->
<-v Update project info -->
<-v Update documentation -->

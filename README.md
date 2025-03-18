# Render

## Overview

Render is a physically based ray tracer project developed as part of Coms 336. The project started with the three books "Ray Tracing in One Weekend," "Ray Tracing: The Next Week," and "Ray Tracing: The Rest of Your Life." It includes various advanced features such as:

- Configurable Camera
- Lambertian reflection
- Metal and glass materials
- Metallic surfaces as mirrors
- Importance sampling
- Bidirectional Reflectance Distribution Functions (BRDFs)
- Shadows
- Probability Density Functions (PDFs)
- Perlin noise
- Quadrilateral and spherical objects

Building on the concepts from these books, I have personally extended the ray tracer to include:
- Trinagle meshes
- Multi-threading (no GPU acceleration)
- Cube maps
- Post-processing effects like bloom and glare
- HDR renders
  
## Features

1. **Materials**:
   - **Lambertian**: Diffuse reflection.
   - **Metal**: Reflective surfaces with varying fuzziness.
   - **Glass**: Refractive surfaces simulating realistic glass.

2. **Geometry**:
   - **Triangle meshes**: Efficient representation of complex shapes.
   - **Quads and spheres**: Simplified geometric primitives.

3. **Lighting and Shadows**:
   - **Importance sampling**: Optimized sampling for light sources.
   - **BRDFs**: Accurate light reflection models.
   - **Shadows**: Realistic shadow casting from objects.

4. **Textures**:
   - **Perlin noise**: Procedural texture generation.
   - **Cube maps**: Environment mapping for reflections.

5. **Post-Processing**:
   - **Bloom and glare**: Enhanced lighting effects.

6. **Performance**:
   - **Multi-threading**: Improved rendering speed using multiple CPU cores.

## How to Use

### Prerequisites

- C++ Compiler (e.g., GCC, Clang)
- CMake (version 3.10 or higher)
- Git (for cloning the repository)

### Building the Project

1. Clone the repository:
   ```sh
   git clone https://github.com/Andrew-Vick/Render.git
   cd Render
2. Create a build directory:
   ```
   cmake -B build
3. Build project using CMake:
   ```
   cmake --build ./build
4. Run the ray tracer:
   ```
   ./inOneWeeked
   

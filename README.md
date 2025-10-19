# Flocking Simulation

Implementation of Olfati-Saber's flocking algorithms from "Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory".

## Features

- Algorithm 1: Basic flocking with fragmentation
- Algorithm 2: Flocking with navigation
- Algorithm 3: Flocking with obstacle avoidance
- Real-time visualization with OpenGL
- Interactive controls

## Building

### Requirements
- C++17 compiler
- CMake 3.10+
- OpenGL
- GLFW

### Build Instructions
```bash
mkdir build && cd build
cmake ..
make
./flocking_simulation
Controls
T - Set target mode

O - Add obstacle mode

C - Clear obstacles

B - Toggle β-agents display

X - Remove target (swarm mode)

G - Toggle connections display

H - Show help

ESC - Exit

Project Structure
main.cpp - Main application and controls

simulation.h/cpp - Flocking algorithms implementation

renderer.h/cpp - OpenGL visualization

Flocking_for_Multi_Agent_...pdf - Original paper

Based On
Reza Olfati-Saber. "Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory". Caltech, 2004.

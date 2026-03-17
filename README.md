# ComplexLab - 3D Physics Simulation Prototype

Welcome to ComplexLab, a 3D experimental environment built with Python and Panda3D. This prototype simulates real-time particle dynamics on complex rail networks using custom physics integration.

## 🌟 Key Features

- **Dynamic Rail System**: Supports both straight rails and curved paths.
- **Real-time Physics Engine**: Uses Euler integration to calculate gravity, friction, and acceleration.
- **Interactive UI**: Real-time analysis of Kinetic Energy, Momentum, and Mechanical Power.
- **Advanced Collision System**: Realistic ball-to-ball and ball-to-rail interactions.
- **Build Mode**: Create and move rails in 3D space with a dynamic cursor.

## 🛠 Installation & Requirements

To run this simulation without bugs, you need **Python 3.10 or higher**.

### 1. Libraries to Install

Open your terminal and run the following command to install all necessary dependencies:

```bash
pip install panda3d numpy wcwidth
```

### 2. File Structure

Ensure your folder is organized as follows:

```
.
├── main.py
└── engine/
    ├── physics.py
    ├── Rail.py
    └── curve_rail.py
```

- **main.py**: The core application
- **engine/physics.py**: Mass and integration logic
- **engine/Rail.py**: Straight rail logic
- **engine/curve_rail.py**: Bezier/Curved rail logic

## 🎮 How to Use (Controls)

| Action | Control |
|--------|---------|
| Rotate Camera | Hold Right-Click + Move Mouse |
| Zoom In/Out | Mouse Wheel |
| Select Particle | Left-Click |
| Move Particle/Rail | Left-Click & Drag |
| Add Particle | Press [P] (at mouse position) |
| Delete Particle | Press [Delete] |
| Build Mode | Press [R] (Click twice for a new rail) |
| Change Build Height | Press [Q] (Up) or [E] (Down) |
| Help Menu | Press [H] |

## 🧪 The Physics Behind ComplexLab

This simulation is based on classical mechanics. The movement of each particle follows Newton's Second Law:

$$\vec{F}_{total} = m \cdot \vec{a}$$

The total force $\vec{F}$ is the sum of:

- **Gravity**: $m \cdot g$ (projected along the rail tangent).
- **Friction**: $-k \cdot \vec{v}$ (opposing the movement).

We use **Euler Integration** to update the state of the system every frame:

- **Velocity update**: $\vec{v}_{new} = \vec{v}_{old} + \vec{a} \cdot dt$
- **Position update**: $\vec{p}_{new} = \vec{p}_{old} + \vec{v} \cdot dt$

## 🏗 Future Roadmap

- Integration of Electromagnetic fields.
- 3D Export of simulation data (CSV/Excel).
- Save/Load system for rail networks.
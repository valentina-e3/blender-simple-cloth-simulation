# Simple Cloth Simulation in Blender

This project implements a cloth simulation using Blender's Python API. Originally developed in 2020 as part of an academic project, it showcases the interaction of external forces, such as gravity and wind, and collisions with objects like spheres, on a simulated cloth. The simulation is built using the mass-spring model and Verlet integration for realistic physics behavior.

## Features

- **Mass-Spring Model**: Represents cloth as a grid of particles connected by springs, with structural and shear springs.

- **Verlet Integration**: Used for stable and efficient motion computation.
- **External Forces**:
    - Gravity
    - Wind (directional)
- **Object Collisions**:
    - Sphere collision handling.
- **Real-Time Animation**: Updates cloth in response to frame changes.

## Usage 

1. Clone this repository.
2. Open Blender and navigate to the Scripting workspace.
3. Open the `simple_cloth_simulation.py` file and execute the script.
4. To render the simulation results, simply uncomment the following code and replace the file path with your desired location. This will generate individual frame images, which you can then compile into a video using your preferred video editing software.

```python
 bpy.context.scene.render.filepath = "C:\\Users\\Valentina\\Desktop\\blender_projects\\collision_and_wind"
 bpy.ops.render.render(animation=True)
```

## Examples

Examples of the rendered simulations can be found in the rendered_animations folder of this repository, provided in MP4 format.
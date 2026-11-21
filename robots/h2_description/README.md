# Unitree H2 description

The MuJoCo model has two entrypoints:

- `H2_loop.xml` contains the robot, its actuators, and six linkage loop constraints. It does not include a floor or scene lighting, so it can be included in another environment or converted as a standalone robot asset.
- `scene.xml` includes `H2_loop.xml` and adds the floor, lighting, textures, and viewer settings previously embedded in the robot model.

Open `scene.xml` instead of `H2_loop.xml` in a MuJoCo viewer to retain the original scene with a ground plane. Existing code that loads `H2_loop.xml` and expects the built-in floor should switch to `scene.xml`.

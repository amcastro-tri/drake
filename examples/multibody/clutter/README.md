# Clutter

To run the default configuration you can simply do (don't forget to start the
visualizer before running the simulation):
```
bazel run examples/multibody/clutter
```

Options can be displayed with:

```
bazel run examples/multibody/clutter -- --help
```

To run with specific options, compile and run with:
```
bazel run examples/multibody/clutter -- --simulator_target_realtime_rate=0.2 --mbp_time_step=1e-2 --objects_per_pile=10 --simulation_time=4.0
```

The most relevant options are:
- `mpb_time_step`: specifies the discrete solver time step.
- `objects_per_pile`: an arbitrary assortment of objects are initialized into
  four piles, each having `objects_per_pile` objects.
- `simulation_time`: The duration of the simulation in seconds.
- `visualize_forces`: Set to `true` to enable the visualization of contact
  forces.

## Contact parameters

This demo uses a compliant point contact model. Model parameters are specified
through the command line flags:
- `box_stiffness`: Linear stiffness (in N/m) for box point contact.
- `sphere_stiffness`: Linear stiffness (in N/m) for sphere point contact.
- `dissipation_time_constant`: Time constant for the linear model of
  dissipation. The same number for all geometries.
- `friction_coefficient`: Coefficient of friction for all geometries
  (dimensionless).

In Drake, a box-box contact would produce a single point of contact. Usually
this point and the contact normal change discontinuously with the relative
position between boxes.

Therefore, to alleviate this problem, this demo allows the user to experiment
with some geometry modeling options that emulate multiple points of contact for
box-box contact:
- `emulate_box_multicontact`: Places an array of `MxM` spheres on each face of the
  box.
- `num_spheres_per_face`: Specifies parameter `M` for `emulate_box_multicontact`.
- `enable_box_box_collision`: Allows to enable/disable box-box collision. As
  discussed, box-box collision is highly discontinuous and non-reliable.
- `add_box_corners`: In addition to face spheres, this option would add spheres
  at each corner of a box.

These options allow the user to get a sense of the importance of continuous or
time-coherent contact queries on the quality of simulation results. In addition,
it allows to perform "scalability tests" by changing parameter
`num_spheres_per_face`.   

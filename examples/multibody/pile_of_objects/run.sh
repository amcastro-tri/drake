#!/bin/sh

time bazel run pile_of_objects -- \
--simulation_time=2 `#General` \
--target_realtime_rate=0 \
--publish_every_time_step=0 \
--mbp_time_step=1e-3 \
`#Integration: bogacki_shampine3,explicit_euler,implicit_euler,semi_explicit_euler,radau1,radau3,runge_kutta2,runge_kutta3,runge_kutta5` \
--integration_scheme=radau3 \
--max_time_step=0.1 \
--accuracy=1e-3 \
--jacobian_scheme=forward \
`#Contact parameters` \
--penetration_allowance=1e-5 \
--stiction_tolerance=1e-4 \
--inclined_plane_coef_kinetic_friction=0.1 \
--bodyB_coef_kinetic_friction=0.1 \
`#Body type: 'sphere', 'block', or 'block_with_4Spheres'` \
--bodyB_type=block_with_4Spheres


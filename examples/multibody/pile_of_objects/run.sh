#!/bin/sh

# Enables spdlog garded with DRAKE_LOGGER_DEBUG() and  DRAKE_LOGGER_TRACE():
# bazel run --copt -UNDEBUG pile_of_objects
time bazel run pile_of_objects -- \
--simulation_time=0.1 `#General` \
--target_realtime_rate=0. \
--publish_every_time_step=0 \
--mbp_time_step=0 \
--objects_per_pile=3 \
--visualize=0 \
`#Integration: bogacki_shampine3,explicit_euler,implicit_euler,semi_explicit_euler,radau1,radau3,runge_kutta2,runge_kutta3,runge_kutta5` \
--integration_scheme=implicit_euler \
--max_time_step=1.e-3 \
--accuracy=1e-2 \
--jacobian_scheme=forward \
--fixed_step=1 \
`#Contact parameters` \
--penetration_allowance=1e-5 \
--stiction_tolerance=1e-2 \
--inclined_plane_coef_kinetic_friction=0.1 \
--bodyB_coef_kinetic_friction=0.1 \
`#Collision geometry type:` \
--only_collision_spheres=1 \
`#SPDlog: debug, trace, warn,unchanged (default)` \
--spdlog_level=unchanged

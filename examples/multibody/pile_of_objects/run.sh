#!/bin/sh

# Enables spdlog garded with DRAKE_LOGGER_DEBUG() and  DRAKE_LOGGER_TRACE():
# bazel run --copt -UNDEBUG pile_of_objects
time bazel run pile_of_objects -- \
--simulation_time=2.0 `#General` \
--target_realtime_rate=0 \
--publish_every_time_step=0 \
--mbp_time_step=0 \
--objects_per_pile=3 \
--visualize=1 \
`#Integration: bogacki_shampine3,explicit_euler,implicit_euler,semi_explicit_euler,radau1,radau3,runge_kutta2,runge_kutta3,runge_kutta5` \
--integration_scheme=implicit_euler \
--max_time_step=1.e-2 \
--accuracy=1.0e-5 \
--jacobian_scheme=forward \
--fixed_step=0 \
`#Contact parameters` \
--penetration_allowance=1e-3 \
--stiction_tolerance=5e-4 \
--friction_coefficient=0.5 \
`#Collision geometry type:` \
--only_collision_spheres=1 \
--num_spheres=3 \
`#SPDlog: debug, trace, warn,unchanged (default)` \
--spdlog_level=unchanged

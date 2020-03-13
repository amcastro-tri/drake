#!/bin/sh

# Enables spdlog garded with DRAKE_LOGGER_DEBUG() and  DRAKE_LOGGER_TRACE():
# bazel run --copt -UNDEBUG pile_of_objects
time bazel run pile_of_objects -- \
--simulation_time=2.0 `#General` \
--simulator_target_realtime_rate=0.2 \
--simulator_publish_every_time_step=0 \
--mbp_time_step=1e-3 \
--objects_per_pile=3 \
--visualize=1 \
`#Integration: bogacki_shampine3,explicit_euler,implicit_euler,semi_explicit_euler,radau1,radau3,runge_kutta2,runge_kutta3,runge_kutta5` \
--simulator_integration_scheme=implicit_euler \
--simulator_max_time_step=1.e-2 \
--simulator_accuracy=1.0e-5 \
--jacobian_scheme=forward \
--fixed_step=1 \
`#Contact parameters` \
--penetration_allowance=1e-4 \
--stiction_tolerance=1e-3 \
--friction_coefficient=5.0 \
`#Collision geometry type:` \
--only_collision_spheres=1 \
--num_spheres=3 \
`#SPDlog: debug, trace, warn,unchanged (default)` \
--spdlog_level=unchanged

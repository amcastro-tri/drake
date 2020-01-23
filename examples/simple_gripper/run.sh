#!/bin/sh

# Enables spdlog garded with DRAKE_LOGGER_DEBUG() and  DRAKE_LOGGER_TRACE():
# bazel run --copt -UNDEBUG pile_of_objects
time bazel run simple_gripper -- \
--simulation_time=0.2 `#General` \
--target_realtime_rate=0.0 \
--publish_every_time_step=0 \
--mbp_discrete_update_period=1e-3 \
--with_viz=1 \
`#Integration: bogacki_shampine3,explicit_euler,implicit_euler,semi_explicit_euler,radau1,radau3,runge_kutta2,runge_kutta3,runge_kutta5` \
--integration_scheme=implicit_euler \
--max_time_step=1.0e-3 \
--accuracy=1.0e-4 \
--jacobian_scheme=forward \
--fixed_step=1 \
--full_newton=1 \
`#Contact parameters` \
--penetration_allowance=1e-4 \
--v_stiction_tolerance=1e-4 \
--ring_friction=0.5 \
`#SPDlog: debug, trace, warn,unchanged (default)` \
--spdlog_level=unchanged
#--friction_coefficient=0.5 \
`#Collision geometry type:` \

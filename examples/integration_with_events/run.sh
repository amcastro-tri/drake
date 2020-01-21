#!/bin/sh

# Enables spdlog garded with DRAKE_LOGGER_DEBUG() and  DRAKE_LOGGER_TRACE():
# bazel run --copt -UNDEBUG pile_of_objects
time bazel run bouncing_ball_run_dynamics -- \
--simulation_time=5.0 `#General` \
--target_realtime_rate=1 \
--publish_every_time_step=0 \
--with_normal_event=0 \
`#Integration: bogacki_shampine3,explicit_euler,implicit_euler,semi_explicit_euler,radau1,radau3,runge_kutta2,runge_kutta3,runge_kutta5` \
--integration_scheme=runge_kutta3 \
--max_time_step=1.e-2 \
--accuracy=1.0e-4 \
--jacobian_scheme=forward \
--fixed_step=1
#`#Contact parameters` \
#--penetration_allowance=1e-3 \
#--stiction_tolerance=5e-4 \
#--friction_coefficient=0.5 \
`#Collision geometry type:` \
#`#SPDlog: debug, trace, warn,unchanged (default)` \
#--spdlog_level=unchanged

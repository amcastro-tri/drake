#!/bin/sh

time bazel run inclined_plane_with_body -- \
--simulation_time=2 `#General` \
--target_realtime_rate=0 \
--publish_every_time_step=0 \
--mbp_time_step=0 \
`#Integration` \
--integration_scheme=implicit_euler \
--max_time_step=0.1 \
--accuracy=1e-3 \
--jacobian_scheme=central \
`#Contact parameters` \
--penetration_allowance=1e-5 \
--stiction_tolerance=1e-4 

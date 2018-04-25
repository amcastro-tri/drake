#!/usr/bin/env bash

# Working parameters
# ** rk2: rk_dt=6e-6 (based on 8 sphere pad), gripper_force=150, finger_width=-0.084
# ** rk3 rk_dt=1e-3, accuracy=1e-3
# ** timestepping: ts_dt=1e-3, gripper_force=42e3
#				   ts_dt=1e-4, gripper_force=85e4
#
${DRAKE_WORKSPACE}/bazel-bin/examples/contact_model/rigid_mug_gripper \
--simulation_type=compliant --ts_dt=1e-4 --gripper_force=150 \
--rk_type=rk2 --rk_dt=6e-6 --accuracy=1e-3 --v_stiction_tolerance=1e-3 \
--ring_samples=8 --pad_depth=3e-3 --ring_orient=0 --ring_youngs_modulus=-1 \
--ring_dissipation=-1 --ring_static_friction=-1 --ring_dynamic_friction=-1 \
--px=0 --py=0 --pz=0 --rx=0 --ry=0 --rz=0 --playback=false \
--finger_width=-0.084

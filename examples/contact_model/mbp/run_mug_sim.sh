#!/usr/bin/env bash

# We need to get into the .runfiles directory in order for sdformat to find root.sdf. See #7874.
cd bazel-bin/examples/contact_model/mbp/rigid_mug_gripper_mbp.runfiles/drake

./examples/contact_model/mbp/rigid_mug_gripper_mbp \
                  --simulation_time=40.0 --target_realtime_rate=0\
                  --gripper_force=10.0 \
                  --integration_scheme=implicit_euler --max_time_step=2e-3 --accuracy=1e-2 \
                  --penetration_allowance=0.01 --v_stiction_tolerance=5e-4 \
                  --ring_static_friction=2.0 --ring_dynamic_friction=1.0 \
                  --ring_samples=8 --pad_depth=3e-3 --ring_orient=0 \
                  --px=0 --py=0 --pz=0 --rx=0 --ry=0 --rz=0 \
                  --finger_width=0.09 #0.08811

# With --finger_width=0.09415, the gripper width just barely touches the mug and it starts falling

cd -

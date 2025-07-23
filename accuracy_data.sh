#!/bin/bash

exe=./bazel-bin/examples/multibody/clutter/clutter

stiffness_vals=("1e7")
margin_vals=("1e-5" "1e-4" "5e-4" "1e-3")
accuracy_vals=("1e-1" "1e-2" "1e-3" "1e-4")
beta_vals=("1")

for beta in "${beta_vals[@]}"; do
  mkdir -p "clutter_data_beta_${beta}";
done

for beta in "${beta_vals[@]}"; do
  for E in "${stiffness_vals[@]}"; do
    for d in "${margin_vals[@]}"; do
      for ac in "${accuracy_vals[@]}"; do
        out_file="clutter_data_beta_${beta}/E_${E}_d_${d}_ac_${ac}.txt"
        echo "Running: E=$E, d=$d, accuracy=$ac, beta=$beta → $out_file"
        $exe \
          --box_stiffness=$E \
          --sphere_stiffness=$E \
          --barrier=$d \
          --margin="1e-3" \
          --beta=$beta \
          --simulator_accuracy=$ac \
          > "$out_file" &
      done
    done
  done
done

wait
echo "All jobs finished."


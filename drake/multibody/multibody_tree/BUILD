# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "multibody_tree_indexes",
    srcs = [],
    hdrs = ["multibody_indexes.h"],
    deps = [],
)

drake_cc_library(
    name = "multibody_tree",
    srcs = [
        "body_node_impl.cc",
        "frame.cc",
        "mobilizer_impl.cc",
        "multibody_tree.cc",
        "revolute_mobilizer.cc",
        "rigid_body.cc",
    ],
    hdrs = [
        "body.h",
        "body_node.h",
        "body_node_impl.h",
        "body_node_welded.h",
        "composite_body_inertias_cache.h",
        "frame.h",
        "mass_properties.h",
        "mobilizer.h",
        "mobilizer_impl.h",
        "multibody_indexes.h",
        "multibody_tree.h",
        "multibody_tree_cache.h",
        "multibody_tree_context.h",
        "multibody_tree_element.h",
        "multibody_tree_topology.h",
        "revolute_mobilizer.h",
        "rigid_body.h",
        "velocity_kinematics_cache.h",
        "weld_mobilizer.h",
    ],
    deps = [
        ":multibody_tree_indexes",
        ":spatial_inertia",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "rotational_inertia",
    srcs = [],
    hdrs = ["rotational_inertia.h"],
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:extract_double",
    ],
)

drake_cc_library(
    name = "spatial_inertia",
    srcs = [],
    hdrs = ["spatial_inertia.h"],
    deps = [
        ":unit_inertia",
        "//drake/math:gradient",
        "//drake/multibody/multibody_tree/math:spatial_algebra",
    ],
)

drake_cc_library(
    name = "unit_inertia",
    srcs = [],
    hdrs = ["unit_inertia.h"],
    deps = [
        ":rotational_inertia",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "multibody_tree_topology_test",
    deps = [":multibody_tree"],
)

drake_cc_googletest(
    name = "mass_properties_test",
    deps = [
        ":spatial_inertia",
    ],
)

drake_cc_googletest(
    name = "double_pendulum",
    deps = [
        ":multibody_tree",
    ],
)

cpplint()

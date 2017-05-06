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
    name = "fem_elements",
    srcs = ["triangle_element.cc"],
    hdrs = [
        "isoparametric_element.h",
        "triangle_element.h",
    ],
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

# === test/ ===

#drake_cc_library(
#    name = "spatial_inertia",
#    srcs = [],
#    hdrs = ["spatial_inertia.h"],
#    deps = [
#        ":unit_inertia",
#        "//drake/math:gradient",
#        "//drake/multibody/multibody_tree/math:spatial_algebra",
#    ],
#)

cpplint()

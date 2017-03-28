# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(
    default_visibility = ["//visibility:public"],
)

#drake_cc_library(
#    name = "spatial_vector",
#    srcs = [],
#    hdrs = ["spatial_vector.h"],
#    deps = [
#        "//drake/common",
#    ],
#)

drake_cc_library(
    name = "spatial_algebra",
    srcs = [],
    hdrs = [
        "small_vectors.h",
        "spatial_algebra.h",
    ],
    deps = [
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "spatial_algebra_tests",
    srcs = ["test/spatial_algebra_tests.cc"],
    deps = [
        ":spatial_algebra",
        "//drake/common:autodiff",
        "//drake/common:nice_type_name",
    ],
)

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

drake_cc_library(
    name = "mesh",
    srcs = [],
    hdrs = [
        "mesh.h",
    ],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "mesh_loader",
    srcs = [],
    hdrs = [
        "mesh_loader.h",
    ],
    deps = [
        ":mesh",
        "//drake/common",
        "//drake/multibody/shapes",
    ],
)

#filegroup(
#    name = "test_models",
#    testonly = 1,
#    srcs = [
#        "//drake/multibody/shapes/test/tri_cube.obj",
#    ],
#)

# === test/ ===

drake_cc_googletest(
    name = "mesh_test",
    data = [
        "//drake/multibody/shapes:test_models",
    ],
    deps = [
        ":mesh",
        ":mesh_loader",
        "//drake/common",
        "//drake/common:drake_path",
    ],
)

cpplint()

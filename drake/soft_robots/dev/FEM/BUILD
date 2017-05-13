# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "shape_functions",
    srcs = ["triangle_shape_function.cc"],
    hdrs = [
        "triangle_shape_function.h",
    ],
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "fem_elements",
    srcs = ["triangle_element.cc"],
    hdrs = [
        "isoparametric_element.h",
        "triangle_element.h",
    ],
    deps = [
        ":shape_functions",
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "fem_solver",
    srcs = ["fem_solver.cc"],
    hdrs = [
        "fem_solver.h",
    ],
    deps = [
        ":fem_elements",
        ":mesh",
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

drake_cc_googletest(
    name = "triangle_element_test",
    deps = [
        ":fem_elements",
        "//drake/common",
    ],
)

#drake_cc_googletest(
#    name = "fem_solver_test",
#    data = [
#        "//drake/multibody/shapes:test_models",
#    ],
#    deps = [
#        ":fem_solver",
#        ":mesh",
#        ":mesh_loader",
#        "//drake/common",
#        "//drake/common:drake_path",
#    ],
#   )

drake_cc_binary(
    name = "fem_solver_integral",
    testonly = 1,
    srcs = ["test/fem_solver_test.cc"],
    data = [
        "//drake/multibody/shapes:test_models",
    ],
    deps = [
        ":fem_solver",
        ":mesh",
        ":mesh_loader",
        "//drake/common",
        "//drake/common:drake_path",
    ],
)

cpplint()

# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

package(
    default_visibility = ["//visibility:public"],
)

# Note that this is only a portion of Bullet.
cc_library(
    name = "bullet",
    srcs = glob([
        "src/BulletCollision/**/*.cpp",
        "src/LinearMath/**/*.cpp",
    ]),
    hdrs = glob([
        "src/BulletCollision/**/*.h",
        "src/LinearMath/**/*.h",
    ]) + ["src/btBulletCollisionCommon.h"],
    copts = ["-Wno-all"],
    defines = ["BT_USE_DOUBLE_PRECISION"],
    includes = ["src"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE.txt"],
    mode = "0644",
    package_dir = "bullet",
)

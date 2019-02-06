# -*- mode: python -*-
# vi: set ft=python :

def wildmagic_repository(
        name):
    native.new_local_repository(
        name = name,
        path = "/home/amcastro/Programming/WildMagic/WildMagic5/SDK",
        build_file = "//tools/workspace/wildmagic:package.BUILD.bazel",
    )

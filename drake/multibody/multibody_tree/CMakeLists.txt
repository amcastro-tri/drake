add_library_with_exports(LIB_NAME drakeMultibodyTree SOURCE_FILES
  body.cc
  joint.cc
  multibody_tree.cc)

target_link_libraries(drakeMultibodyTree
  drakeCommon
  Eigen3::Eigen)
drake_install_libraries(drakeMultibodyTree)
drake_install_headers(
  body.h
  joint.h
  mass_properties.h
  multibody_indexes.h
  multibody_tree.h
  multibody_tree_topology.h)

drake_install_pkg_config_file(drake-multibodytree
  TARGET drakeMultibodyTree
  LIBS -ldrakeMultibodyTree
  REQUIRES
  drake-common
  eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_library_with_exports(LIB_NAME drakeMultibodyTree SOURCE_FILES
  frame.cc
  mobilizer_impl.cc
#  joint.cc
  revolute_mobilizer.cc
  rigid_body.cc
  multibody_tree.cc)

target_link_libraries(drakeMultibodyTree
  drakeCommon
  Eigen3::Eigen)
drake_install_libraries(drakeMultibodyTree)
drake_install_headers(
  body.h
  body_node.h
  body_node_impl.h
#  frame.h
  mobilizer.h
#  mobilizer_context.h
  mobilizer_impl.h
  mass_properties.h
  multibody_tree_context.h
  multibody_tree_element.h
  multibody_indexes.h
  multibody_tree.h
  multibody_tree_cache.h
  multibody_tree_topology.h
  revolute_mobilizer.h
  rigid_body.h
#  soft_body.h
  weld_mobilizer.h)

drake_install_pkg_config_file(drake-multibodytree
  TARGET drakeMultibodyTree
  LIBS -ldrakeMultibodyTree
  REQUIRES
  drake-common
  eigen3)

add_subdirectory(math)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

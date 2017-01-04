
drake_install_headers(
    multibody_indexes.h
    multibody_tree.h)

#if(BUILD_TESTING)
#  add_subdirectory(test)
#endif()

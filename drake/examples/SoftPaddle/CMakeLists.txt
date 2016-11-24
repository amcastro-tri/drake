if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeSoftPaddlePlant
    SOURCE_FILES soft_paddle_plant.cc soft_paddle_state_vector.cc)
  target_link_libraries(drakeSoftPaddlePlant drakeSystemFramework)
  drake_install_libraries(drakeSoftPaddlePlant)
  drake_install_headers(soft_paddle_plant.h soft_paddle_state_vector.h)
  drake_install_pkg_config_file(drake-soft-paddle-plant
    TARGET drakeSoftPaddlePlant
    LIBS -ldrakeSoftPaddlePlant
    REQUIRES
      eigen3)

  add_executable(soft_paddle_run_dynamics soft_paddle_run_dynamics.cc)
  target_link_libraries(soft_paddle_run_dynamics
    drakeSoftPaddlePlant
    drakeRigidBodyPlant
    drakeSystemAnalysis)
  drake_add_test(NAME soft_paddle_run_dynamics
    COMMAND soft_paddle_run_dynamics)
endif()

#if(BUILD_TESTING)
#  add_subdirectory(test)
#endif()


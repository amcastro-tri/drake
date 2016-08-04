set(collision_files collision_element.cc)
list(APPEND collision_files collision_world.cc)

pods_find_pkg_config(bullet)
if(bullet_FOUND)
  add_definitions(-DBULLET_COLLISION -DBT_USE_DOUBLE_PRECISION)
  #set(collision_files ${collision_files} bullet_model.cc)
  list(APPEND collision_files bullet_collision_element.cc
          bullet_collision_world.cc)
  set(bullet "yes" CACHE STRING "yes")
endif()

add_library_with_exports(
        LIB_NAME drakeCollisionEngine SOURCE_FILES ${collision_files})

if(bullet_FOUND)
  pods_use_pkg_config_packages(drakeCollisionEngine bullet)
endif()
target_link_libraries(drakeCollisionEngine drakeShapes drakeCommon)

enable_testing()
add_subdirectory(test)

pods_install_libraries(drakeCollisionEngine)
drake_install_headers(collision_element.h collision_world.h)
if(bullet_FOUND)
  drake_install_headers(bullet_collision_element.h bullet_collision_world.h)
endif()
pods_install_pkg_config_file(drake-collision-engine
  LIBS -ldrakeCollisionEngine
  REQUIRES
  VERSION 0.0.1)

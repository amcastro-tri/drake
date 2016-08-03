if(bullet_FOUND)
  add_executable(collision_test collision_test.cc)
  target_link_libraries(collision_test
          drakeCollisionEngine ${GTEST_BOTH_LIBRARIES})
  add_test(NAME collision_test COMMAND collision_test)

  add_executable(bullet_collision_test bullet_collision_test.cc)
  target_link_libraries(bullet_collision_test drakeCollisionEngine ${GTEST_BOTH_LIBRARIES})
  add_test(NAME bullet_collision_test COMMAND bullet_collision_test)
endif()

cmake_minimum_required(VERSION 3.5)
project(track_odometry)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

ament_auto_add_executable(track_odometry src/track_odometry.cpp)
# ament_auto_add_executable(tf_projection src/tf_projection_node.cpp src/tf_projection.cpp)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  # ament_auto_add_executable(test_tf_projection test/src/test_tf_projection.cpp)
  # target_link_libraries(test_tf_projection ${GTEST_LIBRARIES})
  # ament_auto_add_executable(test_tf_projection_node test/src/test_tf_projection_node.cpp)
  # target_link_libraries(test_tf_projection_node ${GTEST_LIBRARIES})
  # ament_auto_add_executable(test_track_odometry test/src/test_track_odometry.cpp)
  # target_link_libraries(test_track_odometry ${GTEST_LIBRARIES})
endif()

ament_auto_package()

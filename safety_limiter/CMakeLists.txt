cmake_minimum_required(VERSION 3.5)
project(safety_limiter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

ament_auto_add_executable(safety_limiter src/safety_limiter.cpp)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  include_directories(test/include)
  ament_auto_add_executable(test_safety_limiter test/src/test_safety_limiter.cpp)
  target_link_libraries(test_safety_limiter ${GTEST_LIBRARIES})
  ament_auto_add_executable(test_safety_limiter2 test/src/test_safety_limiter2.cpp)
  target_link_libraries(test_safety_limiter2 ${GTEST_LIBRARIES})

  install(DIRECTORY
    test/configs
    DESTINATION share/${PROJECT_NAME}/test/
  )
  add_launch_test(test/test/test_safety_limiter_launch.py
    TARGET test_safety_limiter
    TIMEOUT 120
  )
  add_launch_test(test/test/test_safety_limiter2_launch.py
    TARGET test_safety_limiter2
    TIMEOUT 30
  )
endif()

ament_auto_package()

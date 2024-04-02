cmake_minimum_required(VERSION 3.14)
project(cones_localization)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/cones_localization/cones_localization.hpp
  include/cones_localization/visibility_control.hpp
  src/cones_localization.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/cones_localization/cones_localization_node.hpp
  src/cones_localization_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "cones_localization::ConesLocalizationNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_cones_localization
    test/test_cones_localization.cpp
  )
  ament_target_dependencies(test_cones_localization ${TEST_SOURCES})
  target_link_libraries(test_cones_localization ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

cmake_minimum_required(VERSION 3.8)
project(decision_maker VERSION 0.1.0 LANGUAGES CXX)

# C++20 strict mode
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build flags
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------
# Dependencies
# -------------------------
set(PROJECT_DEPS
  ament_cmake
  ament_index_cpp
  rclcpp
  rclcpp_components
  adore_math
  adore_map
  adore_planning
  adore_ros2_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  adore_dynamics_conversions
  sensor_msgs
  yaml_cpp_vendor
  yaml-cpp
)

foreach(pkg IN LISTS PROJECT_DEPS)
  find_package(${pkg} REQUIRED)
endforeach()

# Common include dirs
function(configure_target_common tgt)
  target_include_directories(${tgt}
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
endfunction()

# -------------------------
# Sources
# -------------------------
set(CORE_SRC
  src/decision_maker.cpp
  src/behaviors.cpp
  src/behaviors_obstacle_avoidance.cpp
  src/conditions.cpp
)

# -------------------------
# Core OBJECT library
# -------------------------
add_library(decision_maker_core OBJECT
  ${CORE_SRC}
)
configure_target_common(decision_maker_core)
ament_target_dependencies(decision_maker_core ${PROJECT_DEPS})

# -------------------------
# Executable node
# -------------------------
add_executable(${PROJECT_NAME}
  src/main.cpp
  $<TARGET_OBJECTS:decision_maker_core>
)
configure_target_common(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPS})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# # -------------------------
# # Component library (without main)
# # -------------------------
# add_library(${PROJECT_NAME}_component SHARED
#   $<TARGET_OBJECTS:decision_maker_core>
# )
# configure_target_common(${PROJECT_NAME}_component)
# ament_target_dependencies(${PROJECT_NAME}_component ${PROJECT_DEPS})
# target_link_libraries(${PROJECT_NAME}_component ${YAML_CPP_LIBRARIES})

# rclcpp_components_register_node(${PROJECT_NAME}_component
#   PLUGIN "adore::DecisionMaker"
#   EXECUTABLE decisionmaker
# )

# install(TARGETS ${PROJECT_NAME}_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # -------------------------
# # Tests
# # -------------------------
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)

#   ament_add_gtest(decision_maker_rules_tests
#     test/test_rules_selection.cpp
#   )
#   configure_target_common(decision_maker_rules_tests)
#   ament_target_dependencies(decision_maker_rules_tests ${PROJECT_DEPS})
#   target_link_libraries(decision_maker_rules_tests
#     ${PROJECT_NAME}_component
#     ${YAML_CPP_LIBRARIES}
#   )
# endif()

ament_package()

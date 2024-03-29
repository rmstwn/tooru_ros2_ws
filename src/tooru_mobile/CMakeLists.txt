cmake_minimum_required(VERSION 3.8)
project(tooru_mobile)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  Eigen3
  eigen3_cmake_module
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# add_executable(kinematics src/kinematics.cpp)
# ament_target_dependencies(kinematics PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(kinematics SHARED src/kinematics.cpp)
ament_target_dependencies(kinematics PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(kinematics "tooru_mobile::TwistToJointPub")

# ament_target_dependencies(kinematics
#   dynamixel_sdk_custom_interfaces
#   dynamixel_sdk
#   rclcpp
#   Eigen3
#   eigen3_cmake_module
#   dynamixel_sdk_custom_interfaces
#   dynamixel_sdk
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# ament_target_dependencies(kinematics Eigen3)
ament_export_dependencies(eigen3_cmake_module)


# install(TARGETS
#   kinematics
#   DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS
    kinematics
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

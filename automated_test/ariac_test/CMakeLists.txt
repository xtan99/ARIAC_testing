cmake_minimum_required(VERSION 3.5)
project(ariac_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp 
  moveit_ros_planning_interface
  ariac_msgs
  shape_msgs
  tf2_kdl 
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add ariac test library
add_library(${PROJECT_NAME} SHARED
  src/ariac_test.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add test_pickparts executable
add_executable(test_pickparts src/test_pickparts.cpp)
target_include_directories(test_pickparts PRIVATE include)
target_link_libraries(test_pickparts
  ${PROJECT_NAME}
)
ament_target_dependencies(test_pickparts ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS test_pickparts
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Add test_picktrays executable
add_executable(test_picktrays src/test_picktrays.cpp)
target_include_directories(test_picktrays PRIVATE include)
target_link_libraries(test_picktrays
  ${PROJECT_NAME}
)
ament_target_dependencies(test_picktrays ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS test_picktrays
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Add test_assembly executable
add_executable(test_assembly src/test_assembly.cpp)
target_include_directories(test_assembly PRIVATE include)
target_link_libraries(test_assembly
  ${PROJECT_NAME}
)
ament_target_dependencies(test_assembly ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS test_assembly
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY 
  config
  launch
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
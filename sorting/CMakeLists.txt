cmake_minimum_required(VERSION 3.5)
project(sorting)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sorting_msgs REQUIRED)

include_directories(include
	${std_msgs_INCLUDE_DIRS}
	${rclcpp_INCLUDE_DIRS}
	${sorting_msgs_INCLUDE_DIRS}
	)

add_library(sorting_lib src/sorting.cpp)

add_executable(sorting src/sorting_node.cpp src/sorting.cpp)

target_link_libraries(sorting
  sorting_lib
	${std_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sorting_msgs_LIBRARIES}
	)

  set(dependencies
  "rclcpp"
  "std_msgs"
  "sorting_msgs"
)

ament_target_dependencies(sorting_lib
  "rclcpp"
  "std_msgs"
  "sorting_msgs"
)

install(TARGETS sorting DESTINATION lib/${PROJECT_NAME})

ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()

set(dependencies
    ${dependencies}
    ${cmr_tests_utils}
)

find_package(cmr_tests_utils REQUIRED)

ament_add_gtest(sorting_test sorting_test.cpp)
target_link_libraries(sorting_test
    ${std_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${sorting_msgs_LIBRARIES}
    sorting_lib
)
ament_target_dependencies(sorting_test ${dependencies})
target_include_directories(sorting_test PRIVATE "../include" "include" ${cmr_tests_utils_INCLUDE_DIRS})
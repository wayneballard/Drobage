# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target roarm_msgs::roarm_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${roarm_msgs_TARGETS}.
if(roarm_msgs_TARGETS AND NOT TARGET roarm_msgs::roarm_msgs)
  add_library(roarm_msgs::roarm_msgs INTERFACE IMPORTED)
  set_target_properties(roarm_msgs::roarm_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${roarm_msgs_TARGETS}")
endif()

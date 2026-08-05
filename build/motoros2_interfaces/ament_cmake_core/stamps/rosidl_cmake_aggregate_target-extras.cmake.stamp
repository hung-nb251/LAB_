# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target motoros2_interfaces::motoros2_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${motoros2_interfaces_TARGETS}.
if(motoros2_interfaces_TARGETS AND NOT TARGET motoros2_interfaces::motoros2_interfaces)
  add_library(motoros2_interfaces::motoros2_interfaces INTERFACE IMPORTED)
  set_target_properties(motoros2_interfaces::motoros2_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${motoros2_interfaces_TARGETS}")
endif()

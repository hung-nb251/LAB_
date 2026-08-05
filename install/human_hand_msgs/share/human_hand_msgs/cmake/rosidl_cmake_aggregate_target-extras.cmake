# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target human_hand_msgs::human_hand_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${human_hand_msgs_TARGETS}.
if(human_hand_msgs_TARGETS AND NOT TARGET human_hand_msgs::human_hand_msgs)
  add_library(human_hand_msgs::human_hand_msgs INTERFACE IMPORTED)
  set_target_properties(human_hand_msgs::human_hand_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${human_hand_msgs_TARGETS}")
endif()

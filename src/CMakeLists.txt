add_subdirectory(plugins)

message("!!!!!!! Generate server ")
add_executable(server_cartpole server_cartpole.cpp)
set_property(TARGET server_cartpole PROPERTY CXX_STANDARD 17)
target_link_libraries(server_cartpole
  PUBLIC ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PUBLIC ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  PUBLIC ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER})

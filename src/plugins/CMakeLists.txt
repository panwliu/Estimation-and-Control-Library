message("!!!!!!! Generate plugin ")
add_library(gazebo_cartpole SHARED gazebo_cartpole.cpp)
set_property(TARGET gazebo_cartpole PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_cartpole
  PUBLIC ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PUBLIC ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  PUBLIC ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER})


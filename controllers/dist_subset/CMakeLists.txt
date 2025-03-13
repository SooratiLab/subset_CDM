add_library(dist_subset SHARED dist_subset.h dist_subset.cpp)
target_link_libraries(dist_subset
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

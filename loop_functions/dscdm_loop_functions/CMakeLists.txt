link_directories(${CMAKE_BINARY_DIR}/controllers/dist_subset)
set(dscdm_loop_functions_SOURCES dscdm_loop_functions.cpp)


add_library(dscdm_loop_functions MODULE ${dscdm_loop_functions_SOURCES})
target_link_libraries(dscdm_loop_functions
  dist_subset
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(dscdm_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)

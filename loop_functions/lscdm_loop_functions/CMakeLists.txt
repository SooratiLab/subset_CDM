link_directories(${CMAKE_BINARY_DIR}/controllers/leader_subset)
set(lscdm_loop_functions_SOURCES lscdm_loop_functions.cpp)


add_library(lscdm_loop_functions MODULE ${lscdm_loop_functions_SOURCES})
target_link_libraries(lscdm_loop_functions
  leader_subset
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(lscdm_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)

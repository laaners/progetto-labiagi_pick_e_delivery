#ia multisolver app, you give a graph, it optimizes it
add_executable(solver_app_star_solver
  solver_app_star_solver.cpp
  app_common.cpp)

target_link_libraries(solver_app_star_solver
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_star_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
  pthread
)

#ia multisolver app, you give a graph, it optimizes it
add_executable(solver_app_star_solver_gui
  solver_app_star_solver_gui.cpp
  app_common.cpp)

target_link_libraries(solver_app_star_solver_gui
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_star_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${CHOLMOD_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CSPARSE_LIBRARY}
  ${catkin_LIBRARIES}
  pthread
)

#ia multisolver app, you give a graph, it optimizes it

add_executable(solver_app_graph_splitter
  solver_app_graph_splitter.cpp)

target_link_libraries(solver_app_graph_splitter
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_star_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(solver_app_star_solver_batch
  solver_app_star_solver_batch.cpp)

target_link_libraries(solver_app_star_solver_batch
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_star_library
  ${SOLVER_EXAMPLES_CORE_LIBRARIES}
  ${catkin_LIBRARIES}
  pthread
)

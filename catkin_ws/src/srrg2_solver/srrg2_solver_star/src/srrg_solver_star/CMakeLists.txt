add_library(srrg2_solver_star_library SHARED
  star_labeler.cpp star_labeler.h
  star_solver.cpp
  star_solver_gauge.cpp
  star_solver_bookkeeping.cpp
  star_solver_optimize.cpp 
  star_solver_internals.cpp
  star_solver_backbone.cpp
  star_solver_merge.cpp star_solver.h
  factor_graph_batch_splitter.cpp factor_graph_batch_splitter.h
  # instances
  instances.cpp instances.h
  )

target_link_libraries(srrg2_solver_star_library
  srrg2_solver_factor_graph_utils_library
  srrg2_solver_incremental_library
  srrg2_solver_types2d_library
  srrg2_solver_types3d_library
  srrg2_solver_projective_library
  srrg2_solver_calib_library
  srrg2_solver_core_library
  )

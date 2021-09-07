#include "instances.h"
#include "srrg_solver/utils/instances.h"
#include "srrg_solver/solver_incremental/instances.h"
#include "star_solver.h"

namespace srrg2_solver {
  void solver_star_registerTypes() {
    solver_utils_registerTypes();
    solver_incremental_registerTypes();
    BOSS_REGISTER_CLASS(StarSolver);
  }
} // namespace srrg2_solver

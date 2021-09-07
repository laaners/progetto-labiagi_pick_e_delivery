#pragma once
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_boss/deserializer.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/solver_incremental/factor_graph_incremental_sorter.h>
#include <srrg_solver/utils/solver_evaluator.h>
#include <srrg_solver/solver_incremental/solver_incremental_running_environment.h>

namespace srrg2_solver {

  struct StarSolverRunningEnvironment : public SolverIncrementalRunningEnvironment{
    StarSolverRunningEnvironment(ParseCommandLine& cmd_line_);
    virtual void endDataCallback();
    virtual void run();
    virtual ~StarSolverRunningEnvironment();
  };

}

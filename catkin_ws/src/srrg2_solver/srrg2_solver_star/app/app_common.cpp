#include <string>
#include <iostream>
#include "app_common.h"
#include "srrg_solver_star/star_solver.h"

namespace srrg2_solver {
  using  namespace std;

  StarSolverRunningEnvironment::StarSolverRunningEnvironment(ParseCommandLine& cmd_line_):
    SolverIncrementalRunningEnvironment(cmd_line_){}

  void StarSolverRunningEnvironment::run() {
    std::set<VariableBase::Id> new_vars;
    std::set<FactorBase::Id> new_factors;
    StarSolverPtr star_solver=dynamic_pointer_cast<StarSolver>(solver);
    while (readEpoch(new_vars, new_factors)) {
      if (star_solver) {
        star_solver->param_do_optinc.setValue(!do_tag.isSet());
        //solver->param_do_merge.setValue(!_do_tag);
        star_solver->param_do_optdown.setValue(!do_tag.isSet());
      }
      if(!new_factors.size())
        continue;
      cerr << "epoch!" << endl;
      startEpochCallback(new_vars, new_factors);
      solver->compute(new_factors, do_tag.isSet());
      endEpochCallback();
    }
    if (new_factors.size()) {
      solver->compute(new_factors, true);
    }
    endDataCallback();
  }

  void StarSolverRunningEnvironment::endDataCallback(){
    Chrono::printReport(solver->resourceUsage());
    if (! output_file.isSet()) {
      cerr << "NO OUTPUT SELECTED, GOODIES NOT SAVED" << endl;
    }

    StarSolverPtr star_solver=dynamic_pointer_cast<StarSolver>(solver);
    if (star_solver) {
      star_solver->backbone()->write(std::string("bbone_")+output_file.value());

      if (!do_finalize.isSet()) {
        star_solver->param_solver_finalize.setValue(nullptr);
      }
    }
      
    cerr << "Finalizing... (might take long)" << endl;
    solver->finalize(); 
    cerr << "done" << endl;
    FactorGraphView final_view;
    final_view.addFactors(*solver->graph(), solver->finalizedFactors());
    final_view.write(std::string("staropt_")+output_file.value());

  }
  StarSolverRunningEnvironment::~StarSolverRunningEnvironment(){}

}

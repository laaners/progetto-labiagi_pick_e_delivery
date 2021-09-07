#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_boss/deserializer.h>
#include <srrg_config/configurable_manager.h>
#include "srrg_solver_star/factor_graph_batch_splitter.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;


const std::string exe_name = "solver_app_graph_initializer";
#define LOG std::cerr << exe_name << "|"


static const char* banner[] = {
  "initializes the factor graph,  attempting first a breadth first on the poses, then a local triangulation of the landmarks",
  "usage: solver_app_graph_initializer -i <input> -o <output>",
  0
};


ConfigurableManager manager;

// ia THE PROGRAM
int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file(&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString config_file(&cmd_line, "c",    "config",             "file where to read the slover configuration ", "splitter.config");
  ArgumentString solver_name(&cmd_line,
                             "sn",
                             "star-solver",
                             "name of the solver in the star",
                             "star_solver");
  ArgumentString output_file(&cmd_line, "o",    "output-file",           "file where to save the output", "");
  ArgumentFlag   do_finalize(&cmd_line, "f", "finalize", "runs a global low level opt when done at sparse level");

  cmd_line.parse();

  if (! input_file.isSet()) {
    cerr << "no input file provided, returning" << std::endl;
    return 0;
  }

  manager.read(config_file.value());
  Chrono::ChronoMap times;
    
  // retrieve the solver 
  StarSolverPtr solver=manager.getByName<StarSolver>(solver_name.value());
  if (!solver) {
    throw std::runtime_error(std::string(environ[0]) + "|ERROR, cannot find solver with name [ " +
                             solver_name.value() + " ] in configuration file [ " +
                             config_file.value() + " ]");
  }
  FactorGraphBatchSplitter splitter;
  splitter.param_star_solver.setValue(solver);
  FactorGraphPtr graph;
  std::cerr << "loding file: [" << input_file.value() << "]... ";
  {
    Chrono("TLOAD: time", &times, false);
    graph = FactorGraph::read(input_file.value());
  }
  std::cerr << "done, factors:" << graph->factors().size() << " vars: " << graph->variables().size() << std::endl;
  std::cerr << "relaxing var 0 ";
  VariableBase* v0=graph->variable(0);
  if (v0) {
    v0->setStatus(VariableBase::Active);
    std::cerr << "OK";
  } else {
    std::cerr << "Fail";
    return 0;
  }
  std::cerr << endl;
  {
    Chrono total("BATCH| time: ", &times, false);
    splitter.param_do_compute.setValue(true);
    solver->setGraph(graph);
    int num_assigned_vars=splitter.compute();
    cerr << "=================== COMPLETED SPARSE STAGE =================== " << endl;
    cerr << "total" << endl
         <<" vars: " << graph->variables().size()
         <<" fact: " << graph->factors().size() << endl;

    cerr << "completed" << endl
         <<" unassigned vars: " << graph->variables().size()-num_assigned_vars
         <<" unassigned fact: " << splitter.originalView().factors().size() << endl;
  }

  if (output_file.isSet()) {
    Chrono c("TSAVE| time: ", &times, false);
    solver->backbone()->write(std::string("bbone_")+output_file.value());
  }
  if (!do_finalize.isSet()) {
    solver->param_solver_finalize.setValue(nullptr);
  }
  cerr << "Finalizing... (might take long)" << endl;
  solver->finalize();
  
  if (output_file.isSet()) {
    Chrono c("TSAVE| time: ", &times, false);
    FactorGraphView final_view;
    final_view.addFactors(*solver->graph(), solver->finalizedFactors());
    final_view.write(std::string("staropt_")+output_file.value());
  }
  cerr << "done" << endl;
  
  cerr << "========================== TIMINGS ============================ " << endl;
  Chrono::printReport(solver->resourceUsage());
  Chrono::printReport(splitter.resourceUsage());
  Chrono::printReport(times);
  return 0;
}

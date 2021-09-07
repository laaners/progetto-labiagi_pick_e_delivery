#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_boss/deserializer.h>
#include "srrg_solver_star/factor_graph_batch_splitter.h"
#include <srrg_config/configurable_manager.h>
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
  cmd_line.parse();

  if (! input_file.isSet()) {
    cerr << "no input file provided, returning" << std::endl;
    return 0;
  }

  manager.read(config_file.value());
    
  // retrieve the solver 
  StarSolverPtr solver=manager.getByName<StarSolver>(solver_name.value());
  if (!solver) {
    throw std::runtime_error(std::string(environ[0]) + "|ERROR, cannot find solver with name [ " +
                             solver_name.value() + " ] in configuration file [ " +
                             config_file.value() + " ]");
  }
  FactorGraphBatchSplitter splitter;
  splitter.param_star_solver.setValue(solver);
  
  std::cerr << "loding file: [" << input_file.value() << "]... ";
  FactorGraphPtr graph = FactorGraph::read(input_file.value());
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
  solver->setGraph(graph);
  int num_assigned_vars=splitter.compute();
  cerr << "=================== COMPLETED PARTITION STAGE =================== " << endl;
  cerr << "total" << endl
       <<" vars: " << graph->variables().size()
       <<" fact: " << graph->factors().size() << endl;

  cerr << "completed" << endl
       <<" unassigned vars: " << graph->variables().size()-num_assigned_vars
       <<" unassigned fact: " << splitter.originalView().factors().size() << endl;
  cerr << "========================== TIMINGS ============================ " << endl;
  Chrono::printReport(splitter.resourceUsage());
  
  cerr << "========================== SAVING ============================= " << endl;
  std::set<Id> written_vars;
  if (! output_file.isSet()) {
    cerr << "no output file is choosen, returning";
    return 0;
  }
  Serializer ser;
  ser.setFilePath(output_file.value());

  cerr << "writing objects" << endl;
  int obcount=0;
  //1. write the parametes
  for (auto p_id: splitter.parameterIds()) {
    VariableBase* v=graph->variable(p_id);
    ser.writeObject(*v);
    written_vars.insert(p_id);
    cerr << "\rcount: " << obcount++;
  }
  EndEpoch end_epoch;
  end_epoch.epoch=-1;
  ser.writeObject(end_epoch);
  for (auto it: splitter.partitionMap()) {
    auto partition = it.second;
    for (auto v_it: partition->variables()) {
      auto v_id=v_it.first;
      if (written_vars.count (v_id))
        continue;
      VariableBase* v=v_it.second;
      written_vars.insert(v_id);
      ser.writeObject(*v);
      cerr << "\rcount: " << obcount++;
    }
    for (auto f_it: partition->factors()) {
      ser.writeObject(*f_it.second);
      cerr << "\rcount: " << obcount++;
    }
    end_epoch.epoch=it.first;
    ser.writeObject(end_epoch);
  }
  cerr << endl << "done" << endl;
  return 0;
}

#include "app_common.h"
#include "srrg_solver_star/star_solver.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;

std::string exe_name;
#define LOG std::cerr << exe_name << "|"

static const char* banner[] = {
  "test program computes the optimization of a factor graph on a hierarchy",
  "the input file should be a sorted graph, use the graph sorter to see",
  "usage: solver_app_incremental_splitter -i <input> -o <output>",
  0
};


// ia THE PROGRAM
int main(int argc, char** argv) {
  exe_name=argv[0];
  ParseCommandLine cmd_line(argv, banner);
  StarSolverRunningEnvironment environment(cmd_line);
  cmd_line.parse();
  std::set<VariableBase::Id> relaxed;
  relaxed.insert(0); // unlock the first node, and let th star solver do its job
  environment.setup(relaxed);
  environment.run();
  return 0;
}

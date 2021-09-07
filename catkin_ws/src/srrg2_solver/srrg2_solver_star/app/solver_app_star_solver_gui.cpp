#include "app_common.h"
#include <srrg_qgl_viewport/viewer_core_shared_qgl.h>
#include <typeinfo>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3.h>
#include "srrg_solver_star/star_solver.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;
using namespace srrg2_qgl_viewport;

const std::string exe_name = "solver_app_incremental_splitter";
#define LOG std::cerr << exe_name << "|"

static const char* banner[] = {
  "test program computes the optimization of a factor graph on a hierarchy",
  "the input file should be a sorted graph, use the graph sorter to see",
  "usage: solver_app_incremental_splitter -i <input> -o <output>",
  0
};

struct StarSolverRunningEnvironmentGUI: public srrg2_solver::StarSolverRunningEnvironment {
  ArgumentFlag use_viewer;
  ArgumentFlag show_neighbors;
  ArgumentFlag show_graph;
  ArgumentFlag show_backbone;
  ArgumentFlag show_pending;
  ArgumentFlag show_trajectory;
  ViewerCanvasPtr global_canvas;
  ViewerCanvasPtr local_canvas;
  VariableBase* last_pose_variable=0;
  Isometry3f last_pose;
  void startEpochCallback(std::set<VariableBase::Id>& new_vars,
                          std::set<VariableBase::Id>& new_factors) override;
  virtual void endEpochCallback() override;

  void showView(ViewerCanvasPtr canvas,
                FactorGraphInterface& view,
                const Isometry3f& viewpoint=Isometry3f::Identity());

  StarSolverRunningEnvironmentGUI(ParseCommandLine& cmd_line);
  ~StarSolverRunningEnvironmentGUI(){}
  
private:
  std::size_t trj_type=0;
};

StarSolverRunningEnvironmentGUI::StarSolverRunningEnvironmentGUI(ParseCommandLine& cmd_line):
  StarSolverRunningEnvironment(cmd_line),
  use_viewer(&cmd_line,
             "v",
             "viewer",
             "show the output using the viewer"),
  show_neighbors(&cmd_line,
                 "sn",
                 "show-neighbors",
                 "show shows the expanded local map"),
  show_graph(&cmd_line,
             "sg",
             "show-graph",
             "shows the full low level graph"),
  show_backbone(&cmd_line,
                "sb",
                "show-backbone",
                "shows the backbone"),
  show_pending(&cmd_line,
               "sp",
               "show-pending",
               "shows the pending factors"),
  show_trajectory(&cmd_line,
                  "st",
                  "show-trajectory",
                  "shows only the trajectory nodes")
{

}


void StarSolverRunningEnvironmentGUI::startEpochCallback(std::set<VariableBase::Id>& new_vars,
                                                         std::set<VariableBase::Id>& new_factors)  {
  srrg2_solver::StarSolverRunningEnvironment::startEpochCallback(new_vars, new_factors);
  if (! new_vars.size())
    return;
  VariableBase* v=graph->variable(*new_vars.begin());
  last_pose_variable=v;
}


void StarSolverRunningEnvironmentGUI::endEpochCallback(){
  StarSolverRunningEnvironment::endEpochCallback();
  if (! global_canvas && !local_canvas)
    return;

  StarSolverPtr star_solver=dynamic_pointer_cast<StarSolver>(solver);

  last_pose.setIdentity();
  VariableSE3Base* v_se3=dynamic_cast<VariableSE3Base*>(last_pose_variable);
  if (v_se3) {
    last_pose=v_se3->estimate();
  }
  
  FactorGraphPtr graph=solver->graph();
  FactorGraphView shown_view;

  // determine the type_id of a backbone edge
  auto global_gauge=solver->globalGauge();
  if (! trj_type && global_gauge) {
    cerr << "trj_type: " << trj_type << endl;
    
    for (auto f_it:graph->factors()) {
      FactorBase* f=f_it.second;
      if (f->numVariables()!=2)
        continue;
      VariableBase* v0=graph->variable(f->variableId(0));
      VariableBase* v1=graph->variable(f->variableId(1));
      if (typeid(*v0)==typeid(*global_gauge)
          && typeid(*v1)==typeid(*global_gauge)){
        trj_type= typeid(*f).hash_code();
        cerr << f->className() << trj_type << endl;
        break;
      }
    }
  }

  if (show_neighbors.isSet()) {
    solver->computeSurroundingView(shown_view);
  }
  if (star_solver && show_backbone.isSet()) {
    shown_view.add(*star_solver->backbone());
  }
  if (show_graph.isSet()) {
    if (star_solver) {
      for (auto s_it:star_solver->stars())
        shown_view.addFactors(*graph, s_it.second->factors_low);
    }
    else {
      shown_view.add(*graph);
    }
  }
  if (star_solver && star_solver->pendingGraph()) {
    shown_view.add(*star_solver->pendingGraph());
  }
  showView(global_canvas, shown_view);
  showView(local_canvas, shown_view, last_pose);
  
}


void StarSolverRunningEnvironmentGUI::showView(ViewerCanvasPtr canvas,
                                               FactorGraphInterface& view,
                                               const Isometry3f& viewpoint) {
  if (! canvas)
    return;
  cerr << "show view" << view.factors().size() << " " << view.variables().size() << endl;
  canvas->pushMatrix();
  canvas->multMatrix(viewpoint.matrix().inverse());
  canvas->pushMatrix();
  canvas->multMatrix(last_pose.matrix());
  canvas->putReferenceSystem(3);
  canvas->popMatrix();
  
  for (auto f_it: view.factors()) {
    FactorBase* f=f_it.second;
    if (! f->enabled())
      continue;
    if (show_trajectory.isSet() && typeid(*f).hash_code()!=trj_type)
      continue;
    f->_drawImpl(canvas);
  }
  for (auto v_it: view.variables()) {
    VariableBase* v=v_it.second;
    if (v->status()!=VariableBase::NonActive)
      v->_drawImpl(canvas);
  }
  canvas->popMatrix();
  
  canvas->flush();
}


void m_runner(StarSolverRunningEnvironmentGUI& environment) {
  environment.run();
}

// ia THE PROGRAM
int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  StarSolverRunningEnvironmentGUI environment(cmd_line);
  cmd_line.parse();
  std::set<VariableBase::Id> relaxed;
  relaxed.insert(0);
  environment.setup(relaxed);
  
  if (environment.use_viewer.isSet()) {
    QApplication qapp(argc, argv);
    ViewerCoreSharedQGL viewer(argc, argv, &qapp);
    environment.global_canvas=viewer.getCanvas("global");
    if (! environment.do_tag.isSet())
      environment.local_canvas=viewer.getCanvas("local");
    std::thread graph_t(m_runner, std::ref(environment));
    viewer.startViewerServer();
    graph_t.join();
  } else {
    environment.run();
  }
  return 0;
}

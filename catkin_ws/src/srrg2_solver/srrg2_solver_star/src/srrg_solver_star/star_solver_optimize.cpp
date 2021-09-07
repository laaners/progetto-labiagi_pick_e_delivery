#include "star_solver.h"
namespace srrg2_solver {
  using namespace std;

  void StarSolver::optimizeUp() {
    if (! _stars.size())
      return;
    setVariablesStatus(*_backbone, VariableBase::Active);
    _global_gauge=_stars.begin()->second->gauge;
    _global_gauge->setStatus(VariableBase::Fixed);
    _initializer_backbone.updateGraph();
    _initializer_backbone.compute();
    cerr << "OPTUP| N_VARS: " << _backbone->variables().size() << " NFACTORS: " << _backbone->factors().size() << endl;
    cerr << "OPTUP| ";
    solve(*_backbone, param_solver_backbone.value(), true);
    _global_gauge->setStatus(VariableBase::Active);
  }
  
  
  void StarSolver::optimizeDown(Star& star_) {
    //VariableBase* gauge=star_.gauge;
    //1. lock the backbone, and initialize the skeleton
    IdSet high_factors;
    IdSet low_factors;


    Star high_view;
    const auto& neighbors=star_.neighbors;
    for (auto n: neighbors) {
      StarPtr s=_stars[n];
      high_view.addFactorsHigh(*s, s->factors_high);
      low_factors.insert(s->factors_low.begin(), s->factors_low.end());
    }
    
    setVariablesStatus(high_view, VariableBase::Active);
    setVariablesStatus(*_backbone, VariableBase::Fixed);
    //2. initialize the skeleton, from the backbone, and solve the rest
    initializeVariables(high_view, param_level.value()+1);
    cerr << "OPTDW| ";
    solve(high_view, param_solver_skeleton.value(),true);
    setVariablesStatus(*_backbone, VariableBase::Active);
  }

  void StarSolver::optimizeIncremental(Star& star) {
    std::set<Star::Id> neighbors;
    for(auto it: star.variables()) {
      auto& s_ids=var2stars(it.first);
      neighbors.insert(s_ids.begin(), s_ids.end());
    }

    //lock all high variables of the neighbors
    IdSet vars_locked;
    for (auto n:neighbors){
      auto s =_stars[n];
      vars_locked.insert(s->vars_high.begin(), s->vars_high.end());
    }

    if (! neighbors.size()) {
      cerr << "no neighbors" << endl;
      cerr << "vars: " << star.variables().size() << endl;
    } else {
      star.is_init=true;
    }
    
    for (auto v_id: vars_locked) {
      auto v = _graph->variable(v_id);
      v->setStatus(VariableBase::Fixed);
    }
    _initializer_incremental.updateGraph();
    _initializer_incremental.compute();
    cerr << "LOCAL| ";
    solve(star, param_solver_stars.value(), true);
    for (auto v_id: vars_locked) {
      auto v = _graph->variable(v_id);
      v->setStatus(VariableBase::Active);
    }
  }

  void StarSolver::finalize() {
    Chrono finalize("FINAL| time:", &_chrono_map, true);
    IdSet backbone_vars;
    VariableBase* gauge=_global_gauge;
    cerr << "FINALIZE| bbone" << endl;
    gauge->setStatus(VariableBase::Fixed);
    solve(*_backbone, param_solver_backbone.value(),true);
    for (auto v_it : _backbone->variables()) {
      backbone_vars.insert(v_it.first);
    }
    
    //1. lock the backbone, and initialize the skeleton
    IdSet high_factors;
    IdSet low_factors;
    
    cerr << "FINALIZE| high [" << endl;
    for (auto s_it:_stars) {
      StarPtr s = s_it.second;
      for (auto f_it: s->factors_high)
        high_factors.insert(f_it.first);
      low_factors.insert(s->factors_low.begin(), s->factors_low.end());
    }
    for (auto v_id: backbone_vars) {
      _backbone->variable(v_id)->setStatus(VariableBase::Fixed);
    }
    
    cerr << "high factors.size(): " << high_factors.size() << endl;
    FactorGraphView high_view;
    high_view.addFactors(*_graph, high_factors);

    cerr << "FINALIZE| high-solve" << endl;
    //2. initialize the skeleton, from the backbone, and solve the rest
    initializeVariables(high_view, param_level.value()+1);
    solve(high_view, param_solver_skeleton.value(),true);
    for (auto v_id: backbone_vars) {
      _backbone->variable(v_id)->setStatus(VariableBase::Active);
    }
    cerr << "FINALIZE| skeleton-solve" << endl;
    gauge->setStatus(VariableBase::Fixed);
    solve(high_view, param_solver_skeleton.value(),true);
    gauge->setStatus(VariableBase::Active);
    _finalized_factors=low_factors;
    if (param_solver_finalize.value()) {
      gauge->setStatus(VariableBase::Fixed);
      cerr << "FINALIZE| low-solve" << endl;
      FactorGraphView low_view;
      low_view.addFactors(*_graph, low_factors);
      for (auto v_it: low_view.variables()) {
        if (! _parameter_ids.count(v_it.first))
          v_it.second->setStatus(VariableBase::Active);
      }
      for (auto f_it: low_view.factors()){
        FactorBase* f = f_it.second;
        if (f->level()==param_level.value())
          f->setEnabled(true);
        else {
          throw std::runtime_error("this sucks");
          f->setEnabled(false);
        }
      }
      gauge->setStatus(VariableBase::Fixed);
      solve(low_view, param_solver_finalize.value(),true);
      gauge->setStatus(VariableBase::Active);
    } else {
      cerr << "no finalize solver, avoiding  low level opt" << endl;
    }
  }

}

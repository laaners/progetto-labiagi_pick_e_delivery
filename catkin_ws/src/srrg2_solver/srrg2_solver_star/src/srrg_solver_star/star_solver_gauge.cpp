#include "star_solver.h"
namespace srrg2_solver {
  using namespace std;


  VariableBase::Id StarSolver::computeGauge(StarPtr star_,
                                            bool force){
    if (param_bf_visit_star.value()) {
      return computeGaugeByVisit(star_, force);
    }
    return computeGaugeByInit(star_, force);
  }
  
  struct IdSort{
    bool operator () (const VariableBase* a, const VariableBase* b) const {return a->graphId()<b->graphId();}
  };
    
  using VariableBaseSet=std::set<VariableBase*, IdSort>;

  VariableBase::Id StarSolver::computeGaugeByInit(StarPtr star_,
                                                  bool force) {
    // assemble potential gauge list
    VariableBaseSet gauge_candidates;
    for (auto v_it: star_->variables()) {
      if (_parameter_ids.count(v_it.first))
        continue;
      // variable should be of gauge type
      if (v_it.second->className()!=param_gauge_type.value())
        continue;
      gauge_candidates.insert(v_it.second);
    }
    if (! gauge_candidates.size()){
      star_->gauge=0;
      return -1;
    }
    std::set<VariableBase*> saved_vars;
    pushVars(saved_vars, *star_);
    setVariablesStatus(*star_, VariableBase::Active);
    int best_init_variables=0;
    VariableBase* best_gauge=0;
    float best_cost=std::numeric_limits<float>::max();
    for (auto this_gauge:gauge_candidates) {
      this_gauge->setStatus(VariableBase::Fixed);
      initializeVariables(*star_, param_level.value());
      this_gauge->setStatus(VariableBase::Active);
      int this_init_variables=_initializer.initializedVariables().size();
      if (best_init_variables>this_init_variables) 
        continue;
      best_init_variables = this_init_variables;
      int this_cost=_initializer.maxCost();
      if (this_cost<0)
        continue;
      if (this_cost<best_cost) {
        best_cost=this_cost;
        best_gauge=this_gauge;
      }
    }
    popVars(saved_vars);
    star_->gauge=best_gauge;
    if (force || best_cost > param_min_cost.value())
      return best_gauge->graphId();
    return -1;
  }

  
  VariableBase::Id StarSolver::computeGaugeByVisit(StarPtr star_,
                                                   bool force) {
    FactorGraphVisit& visit=*param_bf_visit_star.value();
    visit.tainted()=_parameter_ids;
    visit.setGraph(*star_);
    setVariablesStatus(*star_, VariableBase::Active);
    visit.tainted()=_parameter_ids;
    //1. scan the view to seek for a variable good for the gauge
    VariableBase::Id best_gauge_id=-1;
    float best_min_cost=visit.param_max_cost.value()*2;
    for (auto v_it: star_->variables()) {
      // variable should not be in the parameter list
      if (_parameter_ids.count(v_it.first))
        continue;

      // variable should be of gauge type
      if (v_it.second->className()!=param_gauge_type.value())
        continue;
      
      visit.sources().clear();
      visit.sources().insert(v_it.second->graphId());
      visit.compute();
      float this_cost=visit.maxCost();
      //cerr << "( " << v_it.second->graphId() << ": " << this_cost <<  ")";
      if (this_cost<=1)
        continue;
      // graph too small
      if (this_cost<param_min_cost.value()){
        star_->gauge=v_it.second;
        if (! force) {
          best_gauge_id=-1;
          best_min_cost=this_cost;
          break;
        }
      }
      // variable is at lower distance from the rest
      if (this_cost<best_min_cost) {
        star_->gauge=v_it.second;
        best_gauge_id=v_it.second->graphId();
        best_min_cost=this_cost;
      }
    }
    //std::cerr << "cost: " << best_min_cost << endl;
    if (best_gauge_id<0) {
      //star_->clear();
      return best_gauge_id;
    }
    //deactivate all non visited variables
    star_->gauge=star_->variable(best_gauge_id);
    visit.sources().clear();
    visit.sources().insert(best_gauge_id);
    visit.compute();
    int num_non_visited=0;
    for (auto v_it: star_->variables() ){
      int id=v_it.first;
      if (_parameter_ids.count(id))
        continue;
      auto entry = visit.entries().at(id);
      if (! entry) {
        throw std::runtime_error("no entry for var " + std::to_string(id));
      }
    }
    cerr << num_non_visited << endl;
    return best_gauge_id;
  }

}

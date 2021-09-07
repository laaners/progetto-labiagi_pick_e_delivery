#include "star_solver.h"
namespace srrg2_solver {
  using namespace std;

  
  void StarSolver::pushVars(std::set<VariableBase*>& saved_vars, FactorGraphView& s) {
    for (auto v_it: s.variables()){
      VariableBase* v=v_it.second;
      saved_vars.insert(v);
      v->push();
    }
  }
  
  void StarSolver::popVars(std::set<VariableBase*>& saved_vars){
    for (auto v: saved_vars)
      v->pop();
  }

  //activates all variables in the view
  void StarSolver::setVariablesStatus(FactorGraphView& view, VariableBase::Status status) {
    for (auto v_it: view.variables()) {
      if (! _parameter_ids.count(v_it.first))
        v_it.second->setStatus(status);
    }
  }

  void StarSolver::updateActiveFactors(Star& star) {
    int level=param_level.value();
    star.factors_low.clear();
    for (auto f_it: star.factors()){
      FactorBase* f=f_it.second;
      if (f->enabled() && f->level()==level)
        star.factors_low.insert(f->graphId());
    }
  }

  //removes from a view a variable and all factors pointing to it
  void StarSolver::suppressVariable(FactorGraphView& view, VariableBase* v) {
    std::set<FactorBase*> suppressed_factors;
    v->setStatus(VariableBase::NonActive);
    for (auto f_it: view.FactorGraphInterface::factors(v)) {
      FactorBase* f=f_it.second;
      suppressed_factors.insert(f);
    }
    for (auto f: suppressed_factors)
      view.removeFactor(f);
    view.removeVariable(v);
  }
  
  //scans the output of the initializer and disables all vars that are not init
  int StarSolver::suppressUninitializedVariables(FactorGraphInitializer& initializer,
                                                 FactorGraphView& star_) {
    int num_non_init=0;
    int num_init=0;
    std::list<VariableBase*> suppressed;
    for (auto v_it: star_.variables()) {
      VariableBase* v=v_it.second;
      if (v->status()!=VariableBase::Active)
        continue;
      if (! initializer.isInit(v_it.second)) {
        suppressed.push_back(v);
        ++num_non_init;
      } else {
        ++ num_init;
      }
    }
    for (auto v: suppressed) {
      suppressVariable(star_, v);
    }
    cerr << "init: " << num_init  << ", non_init: " << num_non_init;
    return num_non_init;
  }

  
  int StarSolver::suppressOutlierFactors(FactorGraphView& star_) {
    // disable all ouliers in this level
    auto it=param_solver_stars->activeFactorsPerLevel().find(param_level.value());
    const FactorRawPtrVector& active_factors=it->second;
    const FactorStatsVector& factor_stats=param_solver_stars.value()->measurementStats();
    int num_inliers=0;
    std::set<VariableBase*> outlier_variables;
    for (size_t i=0; i<factor_stats.size(); ++i) {
      FactorBase* f = const_cast<FactorBase*>(active_factors[i]);
      f->setRobustifier(0);
      if (factor_stats[i].status!=FactorStats::Status::Inlier) {
        f->setEnabled(false);
        outlier_variables.insert(star_.variable(f->variableId(1)));
      } else {
        ++num_inliers;
      }
    }
    for (auto v: outlier_variables) {
      suppressVariable(star_, v);
    }
    cerr << "inliers: " << num_inliers << ", outliers: " << outlier_variables.size() << ", ";
    return outlier_variables.size();
  }

  int StarSolver::suppressUnderdeterminedVariables(FactorGraphView& star_,
                                                   Solver& solver,
                                                   int level,
                                                   bool verbose) {
    std::list<VariableBase*> suppressed;
    if (verbose) {
      cerr << "suppressing undetermined: " << star_.variables().size() << " " << star_.factors().size() << endl;
    }
    for ( auto v: solver.activeVariables()) {
      if (verbose) cerr << "ptr: " << v << " " << endl;
      if (verbose) cerr << "id: " << v->graphId() << " ";
                                        
      if (v->status()!=VariableBase::Active)
        continue;
      if (verbose) cerr << "lambda: "<< endl;
      float lambda_min, lambda_max;
      bool result=param_solver_stars->lambdaRatio(lambda_min, lambda_max, v,level);
      if (verbose) cerr << "lambda ok "<< endl;
      
      if (! result) {
        suppressed.push_back(v);
      } else {
        float ratio=lambda_min/lambda_max;
        if (ratio< param_lambda_ratio.value()
            || lambda_min < param_lambda_min.value()
            || lambda_max < param_lambda_max.value()) {
          suppressed.push_back(v);
        }
      }
    }
    std::cerr << "undetermined: " << suppressed.size();
    for (auto v: suppressed) {
      suppressVariable(star_, v);
    }
    return suppressed.size();
  }

  
  void StarSolver::initializeVariables(FactorGraphView& view_, int level) {
    _initializer.param_level.setValue(level);
    _initializer.setGraph(view_);
    _initializer.compute();
  }

  bool StarSolver::solve(FactorGraphInterface& star_, SolverPtr solver, bool verbose) {
    if (verbose) std::cerr << solver->name() << ".solve() :" <<" (it/inl/out/supp/chi) ";
    solver->setGraph(star_);
    solver->compute();
    if (verbose) {
      std::cerr << " first:(" 
                << solver->iterationStats()[0].iteration << "/"
                << solver->iterationStats()[0].num_inliers << "/"
                << solver->iterationStats()[0].num_outliers << "/"
                << solver->iterationStats()[0].num_suppressed << "/"
                << solver->iterationStats()[0].chi_inliers << ")";
      std::cerr << " -> last(" 
                << solver->lastIterationStats().iteration << "/"
                << solver->lastIterationStats().num_inliers << "/"
                << solver->lastIterationStats().num_outliers << "/"
                << solver->lastIterationStats().num_suppressed << "/"
                << solver->lastIterationStats().chi_inliers << ")";
      std::cerr << endl;
    }
    return solver->status()==Solver::SolverStatus::Success;
  }

  bool StarSolver::computeMarginals(VariablePairVector& pairs,
                                    MatrixBlockVector& marginals,
                                    FactorGraphView& view,
                                    int level) {
    std::vector<int> max_iterations(level+1, 0);
    max_iterations[level]=1;
    _solver_marginals.param_max_iterations.setValue(max_iterations);
    _solver_marginals.setGraph(view);
    _solver_marginals.suppressPerturbation();
    _solver_marginals.compute();
    if(_solver_marginals.status()!=Solver::SolverStatus::Success)
      return false;
    if (pairs.empty()) {
      pairs.reserve(_solver_marginals.activeVariables().size());
      if (_solver_marginals.status()!=Solver::SolverStatus::Success)
        return false;
      for (auto v:_solver_marginals.activeVariables()) {
        if (v->status()==VariableBase::Active) {
          pairs.emplace_back(std::make_pair(v,v));
        } 
      }
    } else {
      // check that all variables in the pairs are in the active set;
      std::set<VariableBase*> active;
      active.insert(_solver_marginals.activeVariables().begin(),
                    _solver_marginals.activeVariables().end());
      for (auto p: pairs)
        if (! active.count(p.first))
          return false;
    }
    if (! pairs.size())
      return false;
    if (! _solver_marginals.computeMarginalCovariance(marginals, pairs)) {
      throw std::runtime_error("marginals failed");
    }
    return true;
  }

  // creates high edges and variables, and updates the high list
  int StarSolver::labelEdges(Star& star_) {
    VariablePairVector pairs;
    MatrixBlockVector marginals;
    bool marginals_ok=computeMarginals(pairs, marginals, star_, param_level.value());
    if (! marginals_ok) {
      cerr << "labelEdges: cant compute marginals" << endl;
      return false;
    }
    std::cerr << "labels:  ";
    int num_labeled=0;
    star_.factors_high.clear();
    star_.vars_high.insert(star_.gauge->graphId());
    for (size_t i=0; i<marginals.size(); ++i) {
      VariableBase* v=pairs[i].first;
      FactorBasePtr f=_labeler.compute(v, star_.gauge, marginals[i]);
      if(f!=nullptr) {
        f->setEnabled(true);
        f->setLevel(param_level.value()+1);
        f->setGraphId(_start_level_id++);
        star_.addFactor(f.get());
        star_.factors_high[f->graphId()]=f;
        star_.vars_high.insert(v->graphId());
        ++num_labeled;
      }
    }
    cerr << num_labeled <<endl;
    return num_labeled;
  }


  bool StarSolver::makeStar(Star& star_) {
    std::set<VariableBase*> saved_vars;
    pushVars(saved_vars, star_);

    star_.gauge->setStatus(VariableBase::Fixed);
    cerr << "MSTAR| ";
    if (! star_.is_init) {
      initializeVariables(star_, param_level.value());
      suppressUninitializedVariables(_initializer, star_);
    } else {
      suppressUninitializedVariables(_initializer_incremental, star_);
    }
    cerr << endl;

    cerr << "MSTAR| ";
    solve(star_, param_solver_stars.value(), true);
    cerr << "MSTAR| ";
    suppressOutlierFactors(star_);
    suppressUnderdeterminedVariables(star_,
                                     *param_solver_stars.value(),
                                     param_level.value());
    cerr << endl;
    
    cerr << "MSTAR| ";
    bool labels_ok=labelEdges(star_);
    star_.gauge->setStatus(VariableBase::Active);
    updateActiveFactors(star_);
    popVars(saved_vars);
    return labels_ok;
  }

  void StarSolver::updateParameterIds(const IdSet& factor_ids) {
    if (factor_ids.empty()){
      _parameter_ids.clear();
      for (auto v_it: _graph->variables()){
        if (v_it.second->status()==VariableBase::Fixed)
          _parameter_ids.insert(v_it.first);
      }
      return;
    }
    //adjust bookkeeping, scan for parameter ids (which are the variables fixed)
    for (auto f_id: factor_ids) {
      FactorBase* f=_graph->factor(f_id);
      if (! f)
        throw std::runtime_error("no factor in graph");
      for (int i=0; i<f->numVariables(); ++i) {
        auto v_id=f->variableId(i);
        auto v=_graph->variable(v_id);
        if (! v)
          throw std::runtime_error("no var in graph");
        if (v->status()==VariableBase::Fixed)
          _parameter_ids.insert(v_id);
      }
    }
  }



}

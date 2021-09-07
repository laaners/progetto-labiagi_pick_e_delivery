#include <iostream>
#include <Eigen/Eigenvalues>
#include "star_solver.h"
#include "utils.h"

namespace srrg2_solver {
  using namespace std;
  void selectJoiningFactors(IdSet& joining_factors,
                            Star& star,
                            IdSet& common_vars){
    for (auto f_it: star.factors_high) {
      FactorBase* f= f_it.second.get();
      for (int i=0; i<f->numVariables(); ++i) {
        if (common_vars.count(f->variableId(i))) {
          joining_factors.insert(f->graphId());
          break;
        }
      }
    }
  }
  
  void StarSolver::makeBackbone(Star& star_) {
    bool verbose=false;
    if(verbose) cerr << "BACKBONE BEGIN" << endl;
    if(verbose) cerr << "num_stars: " << _stars.size() << endl;
    size_t num_edges=0;
    for (auto s_it: _stars) {
      num_edges+=s_it.second->neighbors.size()+1;
    }
    if(verbose) cerr << "connectivity: " << (double)num_edges/2 << " ratio: " << (double)(num_edges/2)/(double)_stars.size() << endl;
    auto gauge     = star_.gauge;
    //auto gauge_id  = gauge->graphId();
    auto id        = star_.id;
    auto neighbors = star_.neighbors;
    if(verbose) cerr << "neighbor_stats[" << id << "]" << endl;
    _backbone->addVariable(gauge);
    FactorGraphView join_view;
    VariablePairVector target_pair(1);
    int num_new_edges=0;
    //_solver_marginals.param_max_iterations.setValue(_iterations_marginals_backbone);
    for (auto n: neighbors) {
      Star& other_star=*_stars[n];
      VariableBase* other_gauge =  other_star.gauge;
      if (other_gauge==gauge)
        continue;
      std::set<VariableBase::Id> common_vars;
      computeCommonVariables(common_vars, star_, other_star);
      if(verbose) printSet("common_variables: ", common_vars);
      if (! common_vars.size())
        continue;

      IdSet joining_factors;
      selectJoiningFactors(joining_factors, star_, common_vars);
      selectJoiningFactors(joining_factors, other_star, common_vars);
      if(verbose) printSet("joining_factors: ", joining_factors);
      join_view.clear();
      join_view.addFactors(*_graph, joining_factors );
      setVariablesStatus(join_view, VariableBase::Active);
      auto solver=param_solver_skeleton.value();
      if (! solver)
        continue;
      
      std::set<VariableBase*> saved_vars;
      pushVars(saved_vars, join_view);
      gauge->setStatus(VariableBase::Fixed);
      initializeVariables(join_view, param_level.value()+1);
      bool all_ok = solve(join_view, solver);
      all_ok = true;
      target_pair[0]=std::pair(other_gauge, other_gauge);
      MatrixBlockVector marginals;
      if (all_ok)
        all_ok=computeMarginals(target_pair, marginals, join_view, param_level.value()+1);

      if (all_ok) {
        all_ok = ! marginals[0]->isNaN();
      }
      
      if (all_ok) {
        Eigen::MatrixXf eH = marginals[0]->toMatrixXf();
        int dim=marginals[0]->rows();
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf>  es(dim);
        es.compute(eH);
        if (es.eigenvalues()(dim-1)>1e3)
          all_ok=false;
      }

      if (all_ok) {
        FactorBasePtr f=_labeler.compute(other_gauge, gauge, marginals[0]);
        f->setEnabled(true);
        f->setLevel(param_level.value()+2);
        f->setGraphId(_start_level_id++);
        _graph->addFactor(f);
        if (!_backbone->variable(other_gauge->graphId()))
          _backbone->addVariable(other_gauge);
        _backbone->addFactor(f.get());
        ++num_new_edges;
      }
      
      gauge->setStatus(VariableBase::Active);
      popVars(saved_vars);
    }
    if (verbose) cerr << "BACKBONE END" << endl;
    if (! num_new_edges && ! neighbors.size()) {
      throw std::runtime_error("backbone, cannot create any connection to new star, aborting");
    }
  }

}

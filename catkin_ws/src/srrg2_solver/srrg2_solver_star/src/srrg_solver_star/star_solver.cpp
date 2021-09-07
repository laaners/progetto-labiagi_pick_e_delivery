#include <srrg_system_utils/system_utils.h>
#include "srrg_solver/utils/factor_graph_visit_policy.h"
#include "srrg_solver/utils/factor_graph_visit_cost.h"
#include "star_solver.h"
#include <srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_csparse.h>
namespace srrg2_solver {
  using namespace std;
  
  void Star::addFactorsHigh(FactorGraphInterface& src, std::map<FactorBase::Id,   FactorBasePtr>& factors_){
    IdSet factor_ids;
    for (auto f_it: factors_) {
      factor_ids.insert(f_it.first);
    }
    addFactors(src, factor_ids);
  }

  StarSolver::StarSolver(){
    auto cost   = std::shared_ptr<FactorGraphVisitCostUniform>(new FactorGraphVisitCostUniform);
    auto policy = std::shared_ptr<FactorGraphVisitPolicyBase>(new FactorGraphVisitPolicyBase);
    policy->param_cost_function.setValue(cost);
    _uniform_visit.param_cost_policies.pushBack(policy);
    _uniform_visit.param_max_cost.setValue(param_min_cost.value()*2);
    _solver_marginals.param_linear_solver.setValue(SparseBlockLinearSolverPtr(new SparseBlockLinearSolverCholeskyCSparse));
  }

  //sets the initial graph,  clears the skeleton and 
  void StarSolver::setGraph(FactorGraphPtr graph_) {
    _graph=graph_;
    _stars.clear();
    _current_star.reset();
    _initializer_backbone.setGraph(*_backbone);
    if (param_solver_backbone.value()) {
      param_solver_backbone.value()->setGraph(_backbone);
    }
    _global_gauge=0;
  }

  /*
    does one full round of decomposition and, if needed optimization
    it proceeds as follows:

    1. it seeks for a viable gauge, checking that the new portion of the graph has
    a minimum diameter

    2. if a gauge is not found, the computation is postponed
    
    3. pushes all variables (do do a local calculation)

    4. it initializes the variables, having locked the gauge
    and disables all non init vars

    5. attempts a first optimization

    6. suppresses all factors that are outliers, and all vars that are undetermined
    
    7. runs a second optimization, without update using GN to compute the marginals

    8. using the marginal it computes the condensed factors between gauge and all other vars

    9. unlocks gauge and pops all estimates

    10. adjusts the bookkeeping (degree, neighbors, and skeleton)

    11. initializes the skeleton, incrementally

    12. optimizes the skeleton
  */
  
  VariableBase::Id StarSolver::compute(const IdSet& factor_ids, bool force) {
    Chrono time_total("TOTAL| time:", &_chrono_map, true);
    _initializer.parameterIds()=_parameter_ids;

    VariableBase::Id gauge_id=-1;
    //1. find a gauge, if possible
    {
      Chrono time_gauge("GAUGE| time:", &_chrono_map, true);
      updateParameterIds(factor_ids);
      if (! _current_star) {
        _current_star.reset(new Star);
        _initializer_incremental.parameterIds()=_parameter_ids;
        _initializer_incremental.setGraph(*_current_star);
      }
      _current_star->gauge=0;
      _current_star->addFactors(*_graph, factor_ids);
      gauge_id=computeGauge(_current_star, force);
      cerr << "GAUGE| gauge_id: "<< gauge_id << endl;
    }

    //2. incremental optimization
    if (param_do_optinc.value()){
      Chrono time_inc("LOCAL| time:", &_chrono_map, true);
      optimizeIncremental(*_current_star);
    }
    if (gauge_id<0) {
      return gauge_id;
    }

    //3. make a star
    {
      Chrono time_mstar("MSTAR| time:", &_chrono_map, true);
      _current_star->id = _num_stars++;
      _current_star->gauge=_graph->variable(gauge_id);
      if (! makeStar(*_current_star))
        return -1;
    }
    
    //4. update bbone
    {
      Chrono time_bbone("BBONE| time1:", &_chrono_map, true);
      addStar(_current_star, false);
      makeBackbone(*_current_star);
    }
    
    //5. optimize bbone
    {
      Chrono time_optup("OPTUP| time1:", &_chrono_map, true);
      optimizeUp();
    }

    
    //6. try to merge new star with neighobrs
    StarPtr merged_star;
    if (param_do_merge.value()){
      Chrono time_merge("MERGE| time: ", &_chrono_map, true);
      merged_star = merge(_current_star);
    }
    
    //7. if merge happened, updare backbone and optimize upper level
    if (merged_star) {
      _current_star=merged_star;
      {
        Chrono time_bbone("BBONE| time2:", &_chrono_map, true);
        makeBackbone(*_current_star);
      }
      {
        Chrono time_optup("OPTUP| time2:", &_chrono_map, true);
        optimizeUp();
      }
    }

    //8. finally, optimize down based on bbone
    if (param_do_optdown.value()){
      Chrono time_optdw("OPTDW| time:", &_chrono_map, true);
      optimizeDown(*_current_star);

    }
    _last_star=_current_star;
    _current_star.reset();
    return gauge_id;
  }


  void StarSolver::computeSurroundingView(FactorGraphView& view){
    view.clear();
    if (_last_star) { 
      for (auto n: _last_star->neighbors) {
        StarPtr s_other=_stars[n];
        view.addFactors(*_graph, s_other->factors_low);
      }
    }
    if (_current_star) {
      view.add(*_current_star);
    }
  }
}

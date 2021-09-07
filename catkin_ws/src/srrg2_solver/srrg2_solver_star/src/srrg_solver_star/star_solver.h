#pragma once
#include "srrg_solver/solver_incremental/solver_incremental_base.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/utils/factor_graph_visit.h"
#include "srrg_solver/utils/factor_graph_initializer.h"
#include "star_labeler.h"

namespace srrg2_solver{
  using namespace srrg2_core;

  struct Star: public FactorGraphView {
    Id id;
    IdSet vars_high;
    IdSet   factors_low;
    VariableBase*              gauge = 0;
    std::map<FactorBase::Id,   FactorBasePtr>    factors_high;
    bool is_init=false;
    IdSet neighbors;
    void addFactorsHigh(FactorGraphInterface& other, std::map<FactorBase::Id,   FactorBasePtr>& factors_);
  };

  using StarPtr= std::shared_ptr<Star>;
  using IdStarMap = std::map<VariableBase::Id, IdSet >;
  using IdViewMap = std::map<VariableBase::Id, StarPtr>;

  
  
  class StarSolver : public SolverIncrementalBase {
  public:
    StarSolver();
    
    PARAM(PropertyConfigurable_<FactorGraphVisit>,
          bf_visit_star,
          "visit to use for determining the reachable variables",
          0, 0);

    PARAM(PropertyConfigurable_<Solver>,
          solver_stars,
          "solver to compute the initial solution, after init",
          0, 0);

    PARAM(PropertyConfigurable_<Solver>,
          solver_finalize,
          "solver to run the final optimization",
          0, 0);

    PARAM(PropertyConfigurable_<Solver>,
          solver_skeleton,
          "solver to refine the skeleton variables (frontal nodes)",
          0, 0);

    PARAM(PropertyConfigurable_<Solver>,
          solver_backbone,
          "solver to compute the structure of the graph (lumped constraints)",
          0, 0);
  
    PARAM(PropertyFloat,
          min_cost,
          "diameter that if exceeded determines the creation of a new star",
          10.f, 0);

    PARAM(PropertyFloat,
          lambda_ratio,
          "ratio between min and max eval. If below, var is doomed undetermined",
          1e-5, 0);

    PARAM(PropertyFloat,
          lambda_min,
          "min eval of h matrix, if below undedermined",
          1.f, 0);

    PARAM(PropertyFloat, lambda_max,
          "max eval if h, if below undetermined",
          10.f, 0);

    PARAM(PropertyInt,
          level,
          "level of action (lower one)",
          0, &_param_level_changed);

    PARAM(PropertyInt,
          merge_min_shared_vars,
          "minimum number of variables shared between 2 stars to attempt merge",
          5, 0);

    PARAM(PropertyInt,
          merge_distance,
          "distance between gauges to attempt merge",
          3, 0);
    PARAM(PropertyBool,
          do_optinc,
          "if toggled does an incremental optimization",
          true, 0);
    PARAM(PropertyBool,
          do_merge,
          "if toggled attempts the merge",
          true, 0);
    PARAM(PropertyBool,
          do_optdown,
          "if toggled does an optimization down",
          true, 0);

    // call once before the machine runs
    void setGraph(FactorGraphPtr graph_) override;

    // call once, passing the factor pool that is new since the last epoch
    // for comments see the cpp
    VariableBase::Id compute(const IdSet& factor_ids, bool force=false) override;
        
    void finalize() override;
    const IdSet& finalizedFactors() const override {return _finalized_factors;}

    // returns the origin of the global map in the skeleton
    void computeSurroundingView(FactorGraphView& view) override;

    //specific for star solver
    
    // access to local maps, the id is the local map gauge id
    IdViewMap&    stars()       { return _stars;}
    // access to backbone
    FactorGraphViewPtr backbone() {return _backbone;}
    StarPtr lastStar() {return _last_star;}
    StarPtr pendingGraph() {return _current_star;}
protected:
    //! the high level graph containing gauges and frontal vars
    FactorGraphViewPtr _backbone=FactorGraphViewPtr(new FactorGraphView);


    //! map gauge_id -> star at this level
    IdViewMap _stars;

    //! map var_id -> set of id of stars that enclose var_id
    IdStarMap _var_to_star;

    std::map<FactorBase::Id, VariableBase::Id> _factors_to_stars;

    //!starting id for the 1st condensed factor. Must be above the index of
    //!all possible factors in the graph, otherwise bad things happen
    FactorBase::Id _start_level_id=50000000;

    // the functions below are commented in the  cpp
    void removeStar(VariableBase::Id view_id, bool verbose);
    void addStar(StarPtr star_, bool verbose);
    VariableBase::Id computeGauge(StarPtr star_,
                                  bool force);
    VariableBase::Id computeGaugeByInit(StarPtr star_,
                                        bool force);
    VariableBase::Id computeGaugeByVisit(StarPtr star_,
                                         bool force);
    void setVariablesStatus(FactorGraphView& view, VariableBase::Status status);
    void activateFactors(FactorGraphView& view, int level);
    void suppressVariable(FactorGraphView& view , VariableBase* v);
    int suppressUninitializedVariables(FactorGraphInitializer& initializer,
                                       FactorGraphView& view);  
    int suppressOutlierFactors(FactorGraphView& view);
    int suppressUnderdeterminedVariables(FactorGraphView& view,
                                         Solver& solver,
                                         int level,
                                         bool verbose=false);
    int degree(VariableBase::Id id);
    std::set<VariableBase::Id>& var2stars(VariableBase::Id);
    void initializeVariables(FactorGraphView& view_, int level);

    bool solve(FactorGraphInterface& view, SolverPtr solver, bool verbose=false);
    int labelEdges(Star& star_);
    
    // identifies the common vars at high level between two stars
    void computeCommonVariables(IdSet& result,
                                Star& v1,
                                Star& v2);
    void updateParameterIds(const IdSet& factor_ids=
                            IdSet());
    void popVars(std::set<VariableBase*>& saved_vars);
    void pushVars(std::set<VariableBase*>& saved_vars, FactorGraphView& s);
    void updateActiveFactors(Star& star_);
    void makeBackbone(Star& star_);
    bool makeStar(Star& star);
    void optimizeUp();
    void optimizeDown(Star& star_);
    void optimizeIncremental(Star& star_);
    StarPtr merge(StarPtr star_);
    bool updateStar(Star& star, std::map<FactorBase::Id, FactorBasePtr>& factors_in);
    bool computeMarginals(VariablePairVector& pairs,
                          MatrixBlockVector& marginals,
                          FactorGraphView& view,
                          int level);
    void checkNeighbors();
  private:
    // these are temporaries not to be touched until compute has done
    // used to limit the arg list of some functions
    //! current view
    StarPtr _current_star;
    StarPtr _last_star;
    FactorGraphInitializer _initializer; // handy multipurpose initializer
    Solver _solver_marginals;            // to compute marginals in star
    FactorGraphInitializer _initializer_backbone; // incremental initializer for the backbone variables
    Star::Id _num_stars=0;
    //! object to compute the condensed labeling
    StarLabeler _labeler;
    FactorGraphInitializer _initializer_incremental;
    FactorGraphVisit _uniform_visit;
    bool _param_level_changed=true;
    IdSet _finalized_factors;
  };

  using StarSolverPtr=std::shared_ptr<StarSolver>;
  
}

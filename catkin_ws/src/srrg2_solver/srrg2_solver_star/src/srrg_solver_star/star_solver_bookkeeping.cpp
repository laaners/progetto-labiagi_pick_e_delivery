#include "star_solver.h"
#include "utils.h"
namespace srrg2_solver {
  using namespace std;

  
  int StarSolver::degree(VariableBase::Id id){
    auto it=_var_to_star.find(id);
    if (it==_var_to_star.end())
      return 0;
    return it->second.size();
  }
  
  IdSet& StarSolver::var2stars(VariableBase::Id id) {
    auto it=_var_to_star.find(id);
    if (it==_var_to_star.end()) {
      _var_to_star[id]=IdSet();
    }
    return _var_to_star[id];
  }

  void StarSolver::computeCommonVariables(IdSet& result,
                                         Star& v1,
                                         Star& v2) {
    result.clear();
    for (auto v1_id: v1.vars_high) {
      if (v2.vars_high.count(v1_id))
        result.insert(v1_id);
    }
  }

    void StarSolver::checkNeighbors() {
    for (auto n_it: _stars) {
      auto n=n_it.first;
      for (auto& n2: n_it.second->neighbors) {
        if (! _stars[n2]->neighbors.count(n)) {
          throw std::runtime_error("bookkeepin fail"+std::to_string(n2)+" "+std::to_string(n));
        }
      }
    }
  }
  
  //updates the bookkeeping and the skeleton
  //the frontal variables. It also updates the bookkeeping
  void StarSolver::addStar(StarPtr star_, bool verbose) {
    auto id=star_->id;
    if (_stars.count(id)) {
        throw std::runtime_error("existing star id, skip");
    }
    _stars.insert(std::make_pair(id, star_));
    //update
    for (auto v_id: star_->vars_high) {
      var2stars(v_id).insert(id);
    }
    
    for (auto f_it: star_->factors_high) {
      _factors_to_stars[f_it.first]=id;
      _graph->addFactor(f_it.second);
    }

    IdSet& my_neighbors = star_->neighbors;
    my_neighbors.insert(id);
    for (auto v_id: star_->vars_high) {
      if (_parameter_ids.count(v_id))
        continue;
      IdSet& v_stars=var2stars(v_id);
      my_neighbors.insert(v_stars.begin(), v_stars.end());
    }

    // update neighbors of neighbors
    for (auto n: my_neighbors) {
      _stars[n]->neighbors.insert(id);
    }
    IdSet added_vars;
    IdSet   added_factors;
    for (auto v_it: my_neighbors) {
      auto star=_stars[v_it];
      for (auto f_it: star->factors_high) {
        _graph->addFactor(f_it.second);
      }
    }
    checkNeighbors();
  }

    // detaches a view from the structures
  // after that the view is removed from the bookkeping, and its
  // effects on the skeleton are undone
  void StarSolver::removeStar(Star::Id killed_id, bool verbose) {
    if (verbose) std::cerr << endl << "detach " << killed_id << endl;
    StarPtr killed_star=_stars[killed_id];
    VariableBase* killed_gauge=killed_star->gauge;
    VariableBase::Id killed_gauge_id=killed_gauge->graphId();
    
    //1. downdate Degree, and erase the bookkeeping

    if (verbose) cerr << "downdate degree" << endl;
    for (auto v_id: killed_star->vars_high) {
      var2stars(v_id).erase(killed_id);
    }

    for (auto f_it: killed_star->factors_high) {
      _factors_to_stars.erase(f_it.first);
      _graph->removeFactor(f_it.second);
    }

    if (verbose) cerr << "update neighbors" << endl;
    IdSet neighbors=killed_star->neighbors;
    for (auto n: neighbors) {
      StarPtr other_star=_stars[n];
      if (verbose) cerr << "erasing from " << n << " item " << killed_id << endl;
      other_star->neighbors.erase(killed_id);
      if (verbose){
        printSet("neighbors: ", other_star->neighbors);
      }
    }
    if (verbose) cerr << " done" << endl;
    VariableBase* v_bb=_backbone->variable(killed_gauge_id);
    if (verbose) cerr << "BACKBONE_ERASE BEGIN "  << v_bb;
    if (v_bb) {
      std::set<FactorBase*> erased_factors;
      for (auto f_it: _backbone->FactorGraphInterface::factors(v_bb)) {
        erased_factors.insert(f_it.second);
      }
      for (auto f: erased_factors) {
        _backbone->removeFactor(f);
        _graph->removeFactor(f);
      }
      if (verbose) cerr << "VAR_REMOVE BEGIN" << endl;
      _backbone->removeVariable(v_bb);
      if (verbose) cerr << "VAR_REMOVE_END" << endl;
    } 
    if (verbose) cerr << "BACKBONE_ERASE END "  << v_bb;
    if (verbose) cerr << "bind2" << endl;
    if (verbose) cerr << "done" << endl;
    _stars.erase(killed_id);
    if (verbose) cerr << "detach done" << endl;
    checkNeighbors();
  }

}

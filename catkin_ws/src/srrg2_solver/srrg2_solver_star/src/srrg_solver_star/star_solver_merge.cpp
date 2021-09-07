#include "star_solver.h"
namespace srrg2_solver {
  using namespace std;

  StarPtr StarSolver::merge(StarPtr star_){
    SolverPtr solver_skeleton=param_solver_skeleton.value();
    if (! solver_skeleton)
      throw std::runtime_error("Merge| no skeleton solver set");
    Star::Id id = star_->id;
    VariableBase* star_gauge = star_->gauge;
    VariableBase::Id gauge_id=star_gauge->graphId();

    //1. construct a local view, selcting all variables of the two stars
    IdSet temp_candidates;
    IdSet& neighbors=star_->neighbors;
    std::cerr << "MERGE| neighbors: " << gauge_id << " [ ";
    IdSet factors;
    FactorGraphView merge_view;
    for (auto n: neighbors) {
      IdSet common;
      computeCommonVariables(common, *star_, *_stars[n]);
      for (auto v_id: common) {
        VariableBase* v=_graph->variable(v_id);
        if (v->className()==param_gauge_type.value()) {
          factors.insert(_stars[n]->factors_low.begin(),_stars[n]->factors_low.end());
          cerr << n << "(" << "G " <<common.size() << ") ";
          break;
        }
      }
      if ((int)  common.size()>param_merge_min_shared_vars.value() ) {
        temp_candidates.insert(n);
        factors.insert(_stars[n]->factors_low.begin(),_stars[n]->factors_low.end());
        cerr << n << "(" << common.size() << ") ";
      }
    }
    cerr << "] " << endl;
    for (auto f_id: factors){
      if (_graph->factor(f_id)->level()!=param_level.value())
        throw std::runtime_error("level issue");
    }
    if (! factors.size())
      return StarPtr();

    merge_view.addFactors(*_graph, factors);
    //cerr << "merged_graph has " << merge_view->variables().size() << " vars and " << merge_view->factors().size() << " factors" << endl;
    // add all shit to the view
    
    //2. do a visit at low level, and prune the unwanted local maps that are too far
    // compute the distance between the root of the graph and all other gauges
    _uniform_visit.param_max_cost.setValue(param_merge_distance.value()*2);
    _uniform_visit.setGraph(merge_view);
    _uniform_visit.sources().clear();
    _uniform_visit.sources().insert(gauge_id);
    _uniform_visit.tainted()=_parameter_ids;
    _uniform_visit.compute();
    std::set<Star::Id> merged_ids;
    cerr << "MERGE| candidates: " << id << " [ ";
    Star::Id best_candidate=-1;
    float best_cost=1e10;
    merged_ids.insert(id);
    for (auto c: temp_candidates) {
      auto var_id=_stars[c]->gauge->graphId();
      VariableVisitEntry* entry=_uniform_visit.entries().at(var_id);
      cerr << "( " << c << " ";
      if (entry ) {
        cerr << entry->cost << " ";
        if ( entry->cost< param_merge_distance.value() && fabs(c-id)>=param_merge_distance.value() ) {
          cerr << "OK";
          if (c!=id && fabs(c-id)>param_merge_distance.value()) {
            if (entry->cost <  best_cost) {
              best_cost=entry->cost;
              best_candidate=c;
            }
          }
        } else {
          cerr << "NO";
        }
      } else {
        cerr << "inf X" ;
      }
      cerr << ") ";
    }
    cerr << "] " << endl;

    if (best_candidate<0)
      return 0;

    /*3 assemble star*/
    merged_ids.insert(best_candidate);
    IdSet factors_low;
    std::map<FactorBase::Id, FactorBasePtr> factors_high;

    for (auto c: merged_ids) {
      StarPtr s=_stars[c];
      for (auto f_it: s->factors_high) {
        FactorBase* f=f_it.second.get();
        for (int i=0; i<f->numVariables(); ++i) {
          if (! s->vars_high.count(f->variableId(i)))
            throw ("will be a bind error");
        }
      }
      factors_high.insert(s->factors_high.begin(), s->factors_high.end());
      factors_low.insert(s->factors_low.begin(), s->factors_low.end());
    }
    
    
    StarPtr star_merged(new Star);
    star_merged->id=_num_stars++;
    star_merged->factors_low=factors_low;
    star_merged->addFactors(*_graph, factors_low);

    gauge_id=_stars[best_candidate]->gauge->graphId();
    VariableBase* gauge=_graph->variable(gauge_id);
    star_merged->gauge=gauge;
    
    /*add the stuff*/
    if  (! updateStar(*star_merged, factors_high))
      return StarPtr();
    for (auto c: merged_ids)
      removeStar(c, false);
    
    addStar(star_merged, false);
    return star_merged;
  }

  bool StarSolver::updateStar(Star& star_,
                              std::map<FactorBase::Id, FactorBasePtr>& factors_in) {
    for (auto f_it: factors_in) {
      FactorBase* f=f_it.second.get();
      star_.addFactor(f);
    }
    std::set<VariableBase*> saved_vars;
    pushVars(saved_vars, star_);
    
    star_.gauge->setStatus(VariableBase::Fixed);
    cerr << "MERGE| ";
    initializeVariables(star_, param_level.value()+1);
    suppressUninitializedVariables(_initializer, star_);
    cerr << endl;
    bool all_ok=solve(star_, param_solver_skeleton.value());
    if (! all_ok) {
      cerr << "MERGE| fail (solve)" << endl;
      star_.gauge->setStatus(VariableBase::Active);
      popVars(saved_vars);
      return false;
    }
    
    VariablePairVector pairs;
    MatrixBlockVector marginals;
    all_ok=computeMarginals(pairs, marginals, star_, param_level.value()+1);
    if(! all_ok) {
      cerr << "MERGE| fail (marginals)" << endl;
      star_.gauge->setStatus(VariableBase::Active);
      popVars(saved_vars);
      return false;
    }
    for (auto f_it: factors_in) {
      star_.removeFactor(f_it.second.get());
    }
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
    star_.gauge->setStatus(VariableBase::Active);
    popVars(saved_vars);
    return true;
  }

}

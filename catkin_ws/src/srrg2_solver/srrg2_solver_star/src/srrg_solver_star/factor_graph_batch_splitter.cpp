#include "factor_graph_batch_splitter.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_boss/deserializer.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"

#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_point_error_factor.h"

#include "srrg_solver/utils/factor_graph_initializer.h"
#include "srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;
  
  int countCommonVariables(FactorGraphInterface& a,
                           FactorGraphInterface& b,
                           const std::string& gauge_type) {
    int num_common=0;
    for (auto v_it: a.variables()) {
      VariableBase* v=b.variable(v_it.first);
      if (!v)
        continue;
      if (v->status()==VariableBase::Fixed) // skip parameters
        continue;
      if (v->className()==gauge_type)
        return -1; // partitions are connected by a gauge
      ++num_common;
    }
    return num_common;
  }

  int countOpenFactors(FactorGraphInterface& open_view,
                       const IdSet& variable_ids) {
    std::set<FactorBase*> factors;
    for (auto v_id: variable_ids) {
      VariableBase* v=open_view.variable(v_id);
      for(auto f_it: open_view.factors(v)){
        FactorBase* f=f_it.second;
        if (factors.count(f))
          continue;
        for (int i=0; i<f->numVariables(); ++i){
          if (! variable_ids.count(f->variableId(i))) {
            f=0;
            break;
          }
        }
        if (f)
          factors.insert(f);
      }
    }
    return factors.size();
  }


  bool FactorGraphBatchSplitter::Partition::isOpen(FactorGraphView& open_view,
                                                   const std::string& gauge_type) {
    for (auto v_it: variables()) {
      if (v_it.second->className()!=gauge_type)
        continue;
      VariableBase* v=open_view.variable(v_it.first);
      if (!v)
        continue;
      if (open_view.FactorGraphInterface::factors(v).size())
        return true;
    }
    return false;

  }

  void FactorGraphBatchSplitter::Partition::updateNeighbors(IdPartitionMap& partition_map,
                                                            const std::string& gauge_type,
                                                            int min_num_common_vars) {
    for (auto partition_it: partition_map) {
      int other_id=partition_it.first;
      PartitionPtr other_ptr=partition_it.second;
      if (other_id==id) 
        continue;
      int num_common=countCommonVariables(*this, *other_ptr, gauge_type);
      if (num_common<0 || num_common>min_num_common_vars) {
        other_ptr->neighbors.insert(id);
        neighbors.insert(other_id);
      }
    }
  }

  void FactorGraphBatchSplitter::Partition::updateCosts(IdPartitionMap& partition_map) {
    // partitiont the visit from the neighbor of the partition that has the smallest cost
    PartitionPtr root;
    for (auto n: neighbors) {
      if (n==id)
        continue;
      PartitionPtr other=partition_map[n];
      if (! root || other->cost<root->cost) {
        root=other;
      }
    }
    if (! root)
      return;
    PartitionVisitDeque partition_frontier;
    partition_frontier.push(PartitionVisitEntry(root));
    while (! partition_frontier.empty()) {
      PartitionVisitEntry current=partition_frontier.top();
      partition_frontier.pop();
      if (!current.good())
        continue;
      float new_cost=current.cost+1;
      for (auto n: current.partition->neighbors) {
        if (n==current.partition->id)
          continue;
        PartitionPtr other=partition_map[n];
        if (other->cost > new_cost) {
          other->cost=new_cost;
          other->parent=current.partition->id;
          partition_frontier.push(PartitionVisitEntry(other));
        }
      }
    }
  }
  
  Id FactorGraphBatchSplitter::tryInit(IdSet& initialized_vars,
                                       std::set<VariableBase*>& root_candidates) {
    Chrono t_init("TINIT| time: ", &_chrono_map, true);
    _initializer.parameterIds()=_parameter_ids;
    _initializer.param_max_cost.setValue(_max_cost);

    //1. sort the candidates based on the degree
    std::vector<VariableBase*> sorted_candidates(root_candidates.begin(), root_candidates.end());
    std::sort(sorted_candidates.begin(),
              sorted_candidates.end(),
              [&](VariableBase* a, VariableBase* b){
                //return a->graphId()<b->graphId();
                return _original_view.FactorGraphInterface::factors(a).size()
                  > _original_view.FactorGraphInterface::factors(b).size();
              });
  
    //2. attempt an initialization from each of the root variables, stop at the first good one
    int count=0;
    for (auto root_var: sorted_candidates) {
      root_var->setStatus(VariableBase::Fixed);
      _initializer.setGraph(_original_view);
      _initializer.compute();
      root_var->setStatus(VariableBase::Active);
      cerr << "\rTINIT| candidate: " << count ;
      if (countOpenFactors(_original_view, _initializer.initializedVariables())) {
        initialized_vars=_initializer.initializedVariables();
        return root_var->graphId();
      } else {
        _tainted_set.insert(root_var->graphId());
      }
      ++count;
    }
    cerr <<  endl;
    return -1;

  }
  
  Id FactorGraphBatchSplitter::findInitialRoot(IdSet& initialized_vars) {
      std::set<VariableBase*> root_candidates;
      { Chrono this_chrono("FINIT| time:", &_chrono_map, true);
        for (auto v_it: _original_view.variables()) {
          VariableBase* v=v_it.second;
          if (_tainted_set.count(v_it.first))
            continue;
          if (_parameter_ids.count(v->graphId()))
            continue;
          if (v->className()!=_gauge_type)
            continue;
          int degree=_original_view.FactorGraphInterface::factors(v).size();
          if (degree)
            root_candidates.insert(v);
        }
        if (! root_candidates.size())
          return -1;
        cerr << "FINIT| findInitialRoot()  candidates: " << root_candidates.size() << endl;
      }
      return tryInit(initialized_vars, root_candidates);
  }
  
  Id FactorGraphBatchSplitter::findRoot(IdSet& initialized_vars){
    std::set<VariableBase*> root_candidates;
    {
      Chrono this_chrono("FROOT| time:", &_chrono_map, true);
      //1. sort the partitions based on their distance from the initial one
      FactorGraphInitializer initializer;
      std::vector<PartitionPtr> sorted_partitions;
      sorted_partitions.reserve(_open_partitions.size());
      for (auto s_it: _open_partitions) {
        sorted_partitions.emplace_back(s_it.second);
      }
      std::sort(sorted_partitions.begin(),
                sorted_partitions.end(),
                [](const PartitionPtr& a,
                   const PartitionPtr& b)->bool{
                  return a->cost<b->cost;
                });
  
      //2. select the root candidates
      for (auto partition: sorted_partitions) {
        for (auto v_it: partition->variables()) {
          if (_parameter_ids.count(v_it.first))
            continue;
          if (_tainted_set.count(v_it.first))
            continue;
          VariableBase* v=_original_view.variable(v_it.first);
          if (! v)
            continue;
          if (v->className()!=_gauge_type)
            continue;
          root_candidates.insert(v);
        }
      }
    }
    cerr << "PARTN| findRoot, candidates: " << root_candidates.size() << endl;
    return tryInit(initialized_vars, root_candidates);
  }


  int FactorGraphBatchSplitter::compute() {
    StarSolverPtr star_solver=param_star_solver.value();
    if (! star_solver) {
      throw std::runtime_error("no star solver selected");
    }
    if (! star_solver->graph()){
      throw std::runtime_error("no graph selected");
    }

    std::cerr << "CPART| initializing workspace (working view)... ";
    _original_view.clear();
    _original_view.add(*(star_solver->graph()));
    std::cerr << "done" << endl;
    
    std::cerr << "CPART| scanning for parameter_ids... ";
    for(auto v_it: _original_view.variables()){
      if (v_it.second->status()==VariableBase::Fixed)
        _parameter_ids.insert(v_it.first);
    }
    star_solver->parameterIds()=_parameter_ids;
    //std::cerr << printSet(_parameter_ids) << " done" << endl;

    std::cerr << "CPART| synching configurations";
    star_solver->param_bf_visit_star.setValue(nullptr);
    _max_cost=star_solver->param_min_cost.value();
    _min_common_vars=star_solver->param_merge_min_shared_vars.value();
    _gauge_type=star_solver->param_gauge_type.value();
    star_solver->param_do_optinc.setValue(false);
    star_solver->param_do_optdown.setValue(false);
    std::cerr << " done" << endl;
    
    _tainted_set.clear();
    _initializer.parameterIds()=_parameter_ids;
    _open_partitions.clear();
    _partition_map.clear();

    std::cerr << "CPART| start processing" << std::endl;
    Id partition_num=0;
    bool first_round=true;
    while (! _open_partitions.empty() || first_round) {
      std::cerr << "CPART| START -------------- " << partition_num << "--------------" << endl;
      IdSet initialized_vars;
      Id root_id=-1;
      if (first_round) {
        root_id=findInitialRoot(initialized_vars);
        first_round=false;
      } else {
        root_id=findRoot(initialized_vars);
      }
      if (root_id==-1) {
        cerr <<  "no root";
        return _covered_vars.size();
      }
      cerr << "CPART| current root: " << root_id << endl;
      if (initialized_vars.empty()) {
        cerr << "CPART| init set empty" << endl;
        return _covered_vars.size();
      }
      std::cerr << "initialized_vars.size() : " << initialized_vars.size() << endl;
      PartitionPtr partition(new Partition(partition_num));
      partition->addVariables(_original_view, initialized_vars);
      _covered_vars.insert(initialized_vars.begin(), initialized_vars.end());
      std::cerr << "CPART| created new partition " << partition_num
                << " nvars: " << partition->variables().size()
                << " nfactors " << partition->factors().size() << std::endl;
      if (! partition->factors().size() ) {
        _tainted_set.insert(root_id);
        //first_round=true;
        cerr << "CPART| fail,looking for new root" << endl;
        continue;
      }
      // remove from the graph all factors that are in the new view
      for (auto f_it: partition->factors()) {
        _original_view.removeFactor(f_it.second);
      }
      // remove all dangling variables
      for(auto v_id: initialized_vars) {
        VariableBase* v=_original_view.variable(v_id);
        if (_original_view.FactorGraphInterface::factors(v).empty()) {
          _original_view.removeVariable(v);
          //cerr << "removing closed var " << v_id << endl;
        } else {
          //cerr << "leaving open var " << v_id << " degree: " << original_view.FactorGraphInterface::factors(v).size() << endl;
        }
      }
      _partition_map[partition->id]=partition;
      partition->updateNeighbors(_partition_map, _gauge_type, _min_common_vars);
      cerr << "CPART| partition has " << partition->neighbors.size() << " neighbors" << endl;
      partition->updateCosts(_partition_map);
      if (partition->isOpen(_original_view, _gauge_type)) {
        _open_partitions[partition->id]=partition;
        cerr <<  "CPART| partition is open" << endl;
      }
      // remove neighbors from open set
      for (auto n: partition->neighbors) {
        if (n==partition->id)
          continue;
        PartitionPtr other=_partition_map[n];
        if (! other->isOpen(_original_view, _gauge_type)) {
          _open_partitions.erase(other->id);
        }
      }
      cerr << "CPART: #factors_left " << _original_view.factors().size() << endl;
      if (param_do_compute.value()) {
        IdSet partition_factors;
        for (auto f_it: partition->factors())
          partition_factors.insert(f_it.first);
        star_solver->compute(partition_factors, true);
        for (auto v_it: partition->variables()){
          if (_parameter_ids.count(v_it.first))
            continue;
          VariableBase* v=v_it.second;
          v->setStatus(VariableBase::Active);
        }
      }
      std::cerr << "CPART| END -------------- " << partition_num << "--------------" << endl;
      ++partition_num;
    
    }
    cerr << "CPART: end processing " << endl;
    return _covered_vars.size();

  }

}

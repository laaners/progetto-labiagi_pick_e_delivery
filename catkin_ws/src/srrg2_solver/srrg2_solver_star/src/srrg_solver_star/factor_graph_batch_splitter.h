#pragma once
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <Eigen/Cholesky>
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
#include "srrg_solver_star/star_solver.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  class FactorGraphBatchSplitter: public Configurable {
  public:
    PARAM(PropertyConfigurable_<StarSolver>,
          star_solver,
          "solver to compute the star shit",
          0, 0);

    PARAM(PropertyBool,
          do_compute,
          "triggers the computation after each partition",
          false, 0);

    
    struct Partition: public FactorGraphView {
      Partition (Id id_, float cost_=std::numeric_limits<float>::max(), Id parent_=-1) {
        id=id_;
        neighbors.insert(id);
        parent=parent_;
        cost=cost_;
      }
      Id id;
      IdSet neighbors;
      float cost;
      Id parent;

      bool isOpen(FactorGraphView& open_view,
                  const std::string& gauge_type);
  
      void updateNeighbors(std::map<Id, std::shared_ptr<Partition> >& partition_map,
                           const std::string& gauge_type,
                           int min_num_common_vars);

      void updateCosts(std::map<Id, std::shared_ptr<Partition> >& partition_map);
    };

    int compute();
    using PartitionPtr = std::shared_ptr<Partition>;
    using  IdPartitionMap = std::map<Id, PartitionPtr>;
    IdPartitionMap& partitionMap() {return _partition_map;}
    FactorGraphView& originalView() {return _original_view;}
    IdSet& parameterIds() {return _parameter_ids;}
    const Chrono::ChronoMap& resourceUsage() const { return _chrono_map;}
  protected:

    struct PartitionVisitEntry{
      PartitionVisitEntry(PartitionPtr partition_) {
        partition=partition_;
        cost=partition_->cost;
      }
      inline bool operator < (const PartitionVisitEntry& other) const {
        return other.cost<cost;
      }
      inline bool good() const {return cost==partition->cost;}
      PartitionPtr partition;
      float cost;
    };

    using PartitionVisitDeque=std::priority_queue<PartitionVisitEntry>;

    Id tryInit(IdSet& initialized_vars,
               std::set<VariableBase*>& root_candidates);

    Id findInitialRoot(IdSet& initialized_vars);
    
    Id findRoot(IdSet& initialized_vars);
    
    IdPartitionMap _open_partitions;
    IdPartitionMap _partition_map;
    
    IdSet _tainted_set;
    IdSet _parameter_ids;
    IdSet _covered_vars;
    std::string _gauge_type;
    float _max_cost;
    int _min_common_vars;
    FactorGraphView _original_view;
    Chrono::ChronoMap _chrono_map;
    FactorGraphInitializer _initializer;
  };

}

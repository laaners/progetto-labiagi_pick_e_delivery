#pragma once
#include <srrg_solver/variables_and_factors/types_2d/instances.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>
#include <srrg_solver/variables_and_factors/types_projective/variable_sim3_ad.h>
#include <srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h>

namespace srrg2_solver {
  class StarLabeler {
  public:
    StarLabeler();
    FactorBasePtr
    compute(VariableBase* variable_, VariableBase* gauge, MatrixBlockBase* covariance_);

  protected:  
    class StarFactorCreatorBase {
    public:
      virtual FactorBasePtr
      compute(VariableBase* variable_, VariableBase* gauge, MatrixBlockBase* covariance_) = 0;
    };

    using StarFactorCreatorBasePtr = std::shared_ptr<StarFactorCreatorBase>;

    std::vector<StarFactorCreatorBasePtr> _creators;
    template <typename CreatorType>
    void addCreator();
    
  };

    
}

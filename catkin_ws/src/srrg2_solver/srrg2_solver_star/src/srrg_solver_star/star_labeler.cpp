#include "star_labeler.h"
#include <srrg_solver/solver_core/factor_graph.h>
#include <srrg_solver/variables_and_factors/types_2d/all_types.h>
#include <srrg_solver/variables_and_factors/types_3d/all_types.h>
#include <srrg_solver/variables_and_factors/types_projective/all_types.h>
#include <srrg_solver/solver_core/unscented.h>

namespace srrg2_solver{
    
    
  template <typename ErrorFactorType_>
  class StarFactorCreator_ : public StarLabeler::StarFactorCreatorBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Error Factor type
    using FactorType = ErrorFactorType_;
    using FactorPtrType = std::shared_ptr<FactorType>;
    // Extract variable types from the factor
    using VariableTupleType = typename FactorType::VariableTupleType;
    using VariableTypeFrom  = typename VariableTypeAt_<VariableTupleType, 0>::VariableType;
    using VariableTypeTo    = typename VariableTypeAt_<VariableTupleType, 1>::VariableType;
    using FactorInformationMatrixType    = typename FactorType::InformationMatrixType;
    static constexpr int ErrorDim = FactorType::ErrorDim;
    static constexpr int PerturbationDim = VariableTypeTo::PerturbationDim;
    using JacobianType                   = Eigen::Matrix<float, ErrorDim, PerturbationDim>;
    using MeasurementType = typename FactorType::MeasurementType;
    using VariableCovarianceMatrixType   = Eigen::Matrix<float, PerturbationDim, PerturbationDim>;
    const bool use_unscented=true;
    
    FactorBasePtr compute(VariableBase* variable_,
                          VariableBase* gauge_,
                          MatrixBlockBase* covariance_matrix_) override {
      FactorPtrType f(new FactorType);
      VariableTypeFrom* gauge = dynamic_cast<VariableTypeFrom*>(gauge_);
      if (!gauge) {
        return nullptr;
      }
      VariableTypeTo* variable = dynamic_cast<VariableTypeTo*>(variable_);
      if (!variable) {
        return nullptr;
      }
      f->setVariable(0, gauge);
      f->setVariable(1, variable);
      MeasurementType z = gauge->estimate().inverse() * variable->estimate();
      f->setMeasurement(z);

      using UnscentedX=Unscented_<VariableTypeTo::PerturbationDim>;
      FactorInformationMatrixType sigma_z;
      VariableCovarianceMatrixType sigma_xx = covariance_matrix_->eigenType<VariableCovarianceMatrixType>();

      if (use_unscented) {
        UnscentedX unscented_x;
      
        unscented_x.compute(VariableTypeTo::PerturbationVectorType::Zero(), sigma_xx);
        static constexpr int num_points=UnscentedX::NumPoints;

        using UnscentedZ=Unscented_<ErrorDim>;
        typename UnscentedZ::SigmaPoint sigma_points_z[num_points];
        for (int i=0; i<num_points; ++i) {
          variable->push();
          variable->applyPerturbationRaw(&(unscented_x.sigma_points[i].x(0)));
          f->errorAndJacobian();
          sigma_points_z[i].x=f->error();
          sigma_points_z[i].wc=unscented_x.sigma_points[i].wc;
          sigma_points_z[i].wm=unscented_x.sigma_points[i].wm;
          variable->pop();
        }
        typename FactorType::ErrorVectorType mu_e;

        UnscentedZ::recover(mu_e, sigma_z, sigma_points_z, num_points);
      } else {
        f->errorAndJacobian();
        const JacobianType J                = f->template jacobian<1>();
        sigma_z = J * sigma_xx * J.transpose();
      }
      
      Eigen::CompleteOrthogonalDecomposition<FactorInformationMatrixType> dec(sigma_z);
      f->setInformationMatrix(dec.pseudoInverse());
      return f;
    }
  };

  template <typename FactorType>
  void StarLabeler::addCreator() {
    using CreatorType= StarFactorCreator_<FactorType>;
    _creators.push_back(StarFactorCreatorBasePtr(new CreatorType));
  }
  
  StarLabeler::StarLabeler() {
    addCreator< SE3PosePoseGeodesicErrorFactor >();
    addCreator< SE3PosePointErrorFactor >();
    addCreator< SE2PosePoseGeodesicErrorFactor >();
    addCreator< SE2PosePointErrorFactor >();
    addCreator< Sim3PosePoseErrorFactorAD >();
  }

  FactorBasePtr
  StarLabeler::compute(VariableBase* variable_, VariableBase* gauge_, MatrixBlockBase* covariance_) {
    FactorBasePtr f;
    for (auto& creator: _creators) {
      f=creator->compute(variable_, gauge_, covariance_);
      if (f!=nullptr)
        return f;
    }
    return f;
  }
  
}

#pragma once
#include <srrg_config/configurable.h>
#include <srrg_geometry/geometry_defs.h>

namespace srrg2_slam_interfaces {

  template <typename EstimateType_>
  class MotionModelBase_ : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = EstimateType_;
    virtual ~MotionModelBase_() {
    }

    //! retrieves the estimate resulting from the last compute call (or addition)
    const EstimateType& estimate() const {
      return _motion;
    }

    //! computes the most recent estimate based on the buffer
    virtual void compute() = 0;

    //! add a refined pose estimate to the buffer
    virtual void setRobotInLocalMap(const EstimateType& robot_in_local_map_,
                                    const double timestamp_seconds_ = 0.0) = 0;

    //! moves estimate reference in the window (usally to zero) to compensate the offset
    //! this is needed when moving to an new/old local map (tracker estimate is shifted abruptly)
    virtual void shiftTrackerEstimate(const EstimateType& estimate_,
                                      const double timestamp_seconds_ = 0.0) = 0;

    //! adjusts previous estimate (if any)
    virtual void setRobotInLocalMapPrevious(const EstimateType& robot_in_local_map_,
                                            const double timestamp_seconds_ = 0.0) = 0;

    //! resets the motion model to its initial state
    virtual void clear() {
      _motion.setIdentity();
    }

    void reset() override {
      _motion.setIdentity();
    }
  protected:
    EstimateType _motion = EstimateType::Identity();
  };

} // namespace srrg2_slam_interfaces

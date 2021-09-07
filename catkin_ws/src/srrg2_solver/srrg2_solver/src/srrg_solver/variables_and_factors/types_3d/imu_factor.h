#pragma once
#include <srrg_solver/solver_core/ad_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>

namespace srrg2_solver {

  template <typename Scalar_>
  inline Matrix3_<Scalar_> expMap(const Vector3_<Scalar_> omega_) {
    Matrix3_<Scalar_> R;
    Scalar_ theta_square = omega_.dot(omega_);
    Scalar_ theta        = sqrt(theta_square);
    Matrix3_<Scalar_> W  = geometry3d::skew(omega_);
    Matrix3_<Scalar_> K  = W / theta;
    if (theta_square < Scalar_(1e-8)) {
      R = Matrix3_<Scalar_>::Identity() + W;
    } else {
      Scalar_ one_minus_cos = Scalar_(2) * sin(theta / Scalar_(2)) * sin(theta / Scalar_(2));
      R = Matrix3_<Scalar_>::Identity() + sin(theta) * K + one_minus_cos * K * K;
    }
    return R;
  }

  template <typename Scalar_>
  inline Vector3_<Scalar_> logMap(const Matrix3_<Scalar_>& R) {
    const Scalar_ &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
    const Scalar_ &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
    const Scalar_ &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

    // Get trace(R)
    const Scalar_ tr = R.trace();
    const Scalar_ pi(M_PI);
    const Scalar_ two(2);

    Vector3_<Scalar_> omega;
    // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
    // we do something special
    if (tr + Scalar_(1.0) < Scalar_(1e-10)) {
      if (abs(R33 + Scalar_(1.0)) > Scalar_(1e-5)) {
        omega = (pi / sqrt(two + two * R33)) * Vector3_<Scalar_>(R13, R23, Scalar_(1.0) + R33);
      } else if (abs(R22 + Scalar_(1.0)) > Scalar_(1e-5)) {
        omega = (pi / sqrt(two + two * R22)) * Vector3_<Scalar_>(R12, Scalar_(1.0) + R22, R32);
      } else {
        omega = (pi / sqrt(two + two * R11)) * Vector3_<Scalar_>(Scalar_(1.0) + R11, R21, R31);
      }
    } else {
      Scalar_ magnitude;
      const Scalar_ tr_3 = tr - Scalar_(3.0); // always negative
      if (tr_3 < Scalar_(-1e-7)) {
        Scalar_ theta = acos((tr - Scalar_(1.0)) / two);
        magnitude     = theta / (two * sin(theta));
      } else {
        // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
        // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
        magnitude = Scalar_(0.5) - tr_3 * tr_3 / Scalar_(12.0);
      }
      omega = magnitude * Vector3_<Scalar_>(R32 - R23, R13 - R31, R21 - R12);
    }
    return omega;
  }

  class PreintegratedImuMeasurement {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PreintegratedImuMeasurement() {
    }

    void setBiasAcc(const Vector3f& ba_) {
      _bias_acc = ba_;
    }

    void setBiasAngVel(const Vector3f& bw_) {
      _bias_w = bw_;
    }

    Vector3f biasAcceleration() {
      return _bias_acc;
    }

    Vector3f biasAngularVelocity() {
      return _bias_w;
    }

    void reset() {
      _Rz.setIdentity();
      _pz.setZero();
      _vz.setZero();
      _dt = 0.f;
    }

    void preIntegrate(const Vector3f& acc_, const Vector3f& w_, const float& dt_) {
      const Vector3f dtheta = (w_ - _bias_w) * dt_;
      _Rz *= expMap(dtheta);
      _pz += 1.5f * _Rz * (acc_ - _bias_acc) * dt_ * dt_;
      _vz += _Rz * (acc_ - _bias_acc) * dt_;
      _dt += dt_;
    }

    Isometry3f getPoseMeasurement() const {
      Isometry3f Z    = Isometry3f::Identity();
      Z.linear()      = _Rz;
      Z.translation() = _pz;
      return Z;
    }

    Vector3f getVelocityMeasurement() const {
      return _vz;
    }

    float getIntegrationTime() const {
      return _dt;
    }

  protected:
    Vector3f _bias_acc = Vector3f::Zero(); // bias acceleration
    Vector3f _bias_w   = Vector3f::Zero(); // bias angular vel
    Matrix3f _Rz       = Matrix3f::Identity();
    Vector3f _pz       = Vector3f::Zero();
    Vector3f _vz       = Vector3f::Zero();
    float _dt          = 0.f;
  };

  using namespace srrg2_core;
  using VariableVelocityAD = VariablePoint3AD;
  //! @brief pose pose error factor ad that uses quaternion vertices
  class SE3ImuErrorFactor : public ADErrorFactor_<9,
                                                  VariableSE3QuaternionRightAD,
                                                  VariablePoint3AD,
                                                  VariableSE3QuaternionRightAD,
                                                  VariablePoint3AD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType          = ADErrorFactor_<9,
                                    VariableSE3QuaternionRightAD,
                                    VariablePoint3AD,
                                    VariableSE3QuaternionRightAD,
                                    VariablePoint3AD>;
    using VariableTupleType = typename BaseType::VariableTupleType;
    using ADErrorVectorType = typename BaseType::ADErrorVectorType;
    //! @brief how to compute the error
    ADErrorVectorType operator()(VariableTupleType& vars) override {
      const Isometry3_<DualValuef>& Ti = vars.at<0>()->adEstimate(); // from
      const Matrix3_<DualValuef>& Ri   = Ti.linear();
      const Vector3_<DualValuef>& ti   = Ti.translation();
      const Vector3_<DualValuef>& vi   = vars.at<1>()->adEstimate();
      const Isometry3_<DualValuef>& Tj = vars.at<2>()->adEstimate(); // to
      const Matrix3_<DualValuef>& Rj   = Tj.linear();
      const Vector3_<DualValuef>& tj   = Tj.translation();
      const Vector3_<DualValuef>& vj   = vars.at<3>()->adEstimate();

      // prediction step
      Matrix3_<DualValuef> R = Ri.transpose() * Rj;
      Vector3_<DualValuef> t = Ri.transpose() * (tj - ti - vi * _dt);
      Vector3_<DualValuef> v = Ri.transpose() * (vj - vi);
      // std::cerr << "R\n" << R.matrix() << std::endl;
      // std::cerr << "t " << t.transpose() << std::endl;
      // std::cerr << "v " << v.transpose() << std::endl;
      // std::cerr << "dt" << _dt << std::endl;
      ADErrorVectorType error;
      error.segment<3>(0) = t - _inverse_pose_measurement.translation(); // translation error
      Matrix3_<DualValuef> R_error = _inverse_pose_measurement.linear() * R;
      error.segment<3>(3)          = logMap(R_error);      // rotation error axis-angle
      error.segment<3>(6)          = v - _vel_measurement; // velocity error
      return error;
    }

    //! @brief converts the measurement in dual values
    void setMeasurement(const PreintegratedImuMeasurement& pimu_) {
      Isometry3f inv_meas = pimu_.getPoseMeasurement().inverse();
      convertMatrix(_inverse_pose_measurement, inv_meas);
      Vector3f vel_meas = pimu_.getVelocityMeasurement();
      convertMatrix(_vel_measurement, vel_meas);
      _dt = pimu_.getIntegrationTime();
    }

  protected:
    //! @brief measurement
    Isometry3_<DualValuef> _inverse_pose_measurement;
    Vector3_<DualValuef> _vel_measurement;
    DualValuef _dt;
  };

} // namespace srrg2_solver
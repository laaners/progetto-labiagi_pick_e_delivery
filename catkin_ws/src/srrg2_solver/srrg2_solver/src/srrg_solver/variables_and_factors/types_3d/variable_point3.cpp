#include "variable_point3.h"
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"
#include <srrg_geometry/geometry3d.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  void VariablePoint3::setZero() {
    this->_updated = true;
    _estimate.setZero();
  }

  void VariablePoint3::applyPerturbation(const Vector3f& pert) {
    this->_updated = true;
    _estimate += pert;
  }

  void VariablePoint3::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("VariablePoint3::draw|invalid canvas");
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fGreen());
    canvas_->putPoints(1, &_estimate);
    canvas_->popAttribute();
  }

  INSTANTIATE(VariablePoint3)
} // namespace srrg2_solver

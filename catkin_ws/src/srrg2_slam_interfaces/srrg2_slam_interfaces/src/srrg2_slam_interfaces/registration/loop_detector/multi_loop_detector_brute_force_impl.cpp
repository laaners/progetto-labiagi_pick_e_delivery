#pragma once
#include "multi_loop_detector_brute_force.h"
#include "srrg2_slam_interfaces/mapping/local_map.h"
#include <srrg_system_utils/shell_colors.h>

namespace srrg2_slam_interfaces {

#define DEBUG(var) \
  if (var)         \
  std::cerr

  static bool loop_detector_debug = true;

  template <typename SLAMAlgorithmType_, typename AlignerType_>
  void MultiLoopDetectorBruteForce_<SLAMAlgorithmType_, AlignerType_>::compute() {
    if (!ThisType::_slam) {
      throw std::runtime_error("MultiLoopDetectorBruteForce_::compute| no slam selected");
    }

    FactorGraphInterfacePtr graph = ThisType::_slam->graph();
    if (!graph) {
      throw std::runtime_error("MultiLoopDetectorBruteForce_::compute| _graph is NULL");
    }
    LocalMapType* source_local_map = ThisType::_slam->currentLocalMap();
    if (!source_local_map) {
      throw std::runtime_error("MultiLoopDetectorBruteForce_::compute| _current_local_map is NULL");
    }

    using ClosureHint       = typename LocalMapSelectorType::ClosureHint;
    using ClosureHintPtr    = typename LocalMapSelectorType::ClosureHintPtr;
    using ClosureHintPtrSet = typename LocalMapSelectorType::ClosureHintPtrSet;

    ClosureHintPtrSet hints;

    ThisType::_attempted_closures.clear();
    if (ThisType::param_local_map_selector.value()) {
      ThisType::param_local_map_selector->setSLAMAlgorithm(ThisType::_slam);
      ThisType::param_local_map_selector->compute();
      hints = ThisType::param_local_map_selector->hints();
    } else {
      for (auto it : graph->variables()) {
        if (it.second != source_local_map) {
          LocalMapType* m = dynamic_cast<LocalMapType*>(it.second);
          hints.insert(ClosureHintPtr(new ClosureHint(m)));
        }
      }
    }

    ThisType::_detected_closures.clear();
    std::shared_ptr<AlignerType> aligner = param_relocalize_aligner.value();
    if (!aligner) {
      throw std::runtime_error("MultiLoopDetectorBruteForce_::compute| no aligner");
    }
    DEBUG(loop_detector_debug) << "LoopDetectorBruteForce_::loopDetect|search: "
                               << FG_BGREEN(source_local_map->graphId()) << "  [\n";
    const EstimateType& pose_in_current =
      ThisType::_slam->robotInLocalMap(); // pose in the current map

    // DEBUG(loop_detector_debug) << "fixed set" << std::endl;
    // for (auto it:_current_local_map->dynamic_properties.properties()){
    //   DEBUG(loop_detector_debug) << "p: " << it.first << std::endl;
    // }
    aligner->setFixed(&source_local_map->dynamic_properties);
    for (auto& h : hints) {
      // srrg this is done in the loop selector
      //      if (h->local_map == source_local_map) {
      //        continue;
      //      }
      LocalMapType* target_local_map = h->local_map;
      if (!target_local_map) {
        continue;
      }
      ThisType::_attempted_closures.insert(target_local_map);

      DEBUG(loop_detector_debug) << "(" << target_local_map->graphId();
      aligner->setMoving(&target_local_map->dynamic_properties);
      aligner->setMovingInFixed(h->initial_guess);

      aligner->compute();
      if (aligner->status() != AlignerBase::Success) {
        DEBUG(loop_detector_debug)
          << ", " << FG_BRED("ALIGNER DROP [code: " << aligner->status() << "]") << ")\n";
        continue;
      }
      // try to localize the platform

      const IterationStats& istat = aligner->iterationStats().back();

      const size_t num_correspondences = aligner->numCorrespondences();
      const size_t num_inliers         = istat.num_inliers;
      const float chi_inliers          = istat.chi_inliers / num_inliers;

      // srrg this should be done in the slice
      if (num_inliers < param_relocalize_min_inliers.value()) {
        DEBUG(loop_detector_debug) << ", " << FG_BRED("NUM_INLIERS DROP: ") << num_inliers << ")\n";
        continue;
      }

      if (chi_inliers > param_relocalize_max_chi_inliers.value()) {
        DEBUG(loop_detector_debug)
          << ", " << FG_BRED("MAX_CHI_INLIERS DROP: ") << chi_inliers << ")\n";
        continue;
      }

      float inlier_ratio = (float) num_inliers / (float) num_correspondences;
      DEBUG(loop_detector_debug) << ", " << inlier_ratio;
      if (inlier_ratio < param_relocalize_min_inliers_ratio.value()) {
        DEBUG(loop_detector_debug)
          << ", " << FG_BRED("MIN_INLIERS_RATIO DROP") << inlier_ratio << ")\n";
        continue;
      }

      DEBUG(loop_detector_debug) << ", " << FG_BGREEN("ACCEPT") << ")\n";
      else {
        std::cerr << FG_BGREEN("ACCEPT") << std::endl;
      }
      using InformationMatrixType = typename LoopClosureType::InformationMatrixType;

      // we add a factor to the map
      EstimateType pose_in_target = aligner->movingInFixed().inverse() * pose_in_current;

      std::shared_ptr<LoopClosureType> closure(
        new LoopClosureType(-1, // ThisType::_slam->generateGraphId(),
                            source_local_map,
                            target_local_map,
                            aligner->movingInFixed(),
                            InformationMatrixType::Identity(),
                            pose_in_target,
                            chi_inliers,
                            num_inliers,
                            num_correspondences));
      ThisType::_detected_closures.push_back(closure);
    }
    DEBUG(loop_detector_debug) << "]" << std::endl;
  }


  template <typename SLAMAlgorithmType_, typename AlignerType_>
  void MultiLoopDetectorBruteForce_<SLAMAlgorithmType_, AlignerType_>::reset() {
    if (param_relocalize_aligner.value())
      param_relocalize_aligner->reset();
  }

} // namespace srrg2_slam_interfaces

#undef DEBUG

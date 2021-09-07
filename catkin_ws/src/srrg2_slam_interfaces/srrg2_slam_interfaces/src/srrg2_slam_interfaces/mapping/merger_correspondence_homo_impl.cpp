#include "merger_correspondence_homo.h"

#include <unordered_set>
/*
#define SPECIALIZE(EstimateType_, SceneType_) \
  template void MergerCorrespondenceHomo_<EstimateType_, SceneType_>::compute()
*/
namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename EstimateType_, typename SceneType_>
  void MergerCorrespondenceHomo_<EstimateType_, SceneType_>::compute() {
    assert(ThisType::_scene);
    assert(ThisType::_measurement);
    ThisType::_status = MergerBase::Initializing;

    // ds check if we can skip computation
    if (!ThisType::_scene_changed_flag && !ThisType::_meas_changed_flag &&
        !ThisType::_correspondences_changed_flag && !ThisType::_meas_in_scene_changed_flag) {
      ThisType::_status = MergerBase::Success;
      return;
    }
    // ds cache
    const float& maximum_response            = param_maximum_response.value();
    const float& maximum_distance_geometry   = param_maximum_distance_geometry_squared.value();
    const EstimateType& measurement_in_scene = ThisType::_measurement_in_scene;
    const size_t number_of_points_meas       = ThisType::_measurement->size();
    const size_t number_of_points_scene      = ThisType::_scene->size();

    // ds if no correspondences are set (usually for initial frame) TODO think about this
    if (!ThisType::_correspondences) {
      // ds we have to initialize the scene with all points in moving
      ThisType::_scene->reserve(number_of_points_scene + number_of_points_meas);
      // ds TODO perform inner collision check?
      for (PointType point_moving : *ThisType::_measurement) {
        if (point_moving.status == Valid) {
          // ds transform and add new point to scene
          point_moving.template transformInPlace<Isometry, EstimateType>(measurement_in_scene);
          ThisType::_scene->emplace_back(point_moving);
        }
      }
    } else {
      if (ThisType::_correspondences->empty()) {
        std::cerr << "MergerCorrespondenceHomo::compute|WARNING: no correspondences" << std::endl;
      }
      assert(!ThisType::_scene->empty());
      assert(!ThisType::_measurement->empty());

      // ds for all correspondences - merge points - scene size does not change!
      std::unordered_set<size_t> merged_points_moving;
      for (const Correspondence& correspondence : *ThisType::_correspondences) {
        assert(static_cast<size_t>(correspondence.fixed_idx) < number_of_points_scene);
        assert(static_cast<size_t>(correspondence.moving_idx) < number_of_points_meas);
        PointType& point_scene      = (*ThisType::_scene)[correspondence.fixed_idx];
        const PointType& point_meas = (*ThisType::_measurement)[correspondence.moving_idx];
        assert(correspondence.response >= 0);
        assert(point_scene.status == Valid);
        assert(point_meas.status == Valid);
        // ds before transforming a point we check the matching distance of the correspondence
        if (correspondence.response < maximum_response) {
          // ds transform moving point into scene coordinate frame (moving is const so no in-place)
          const VectorType coordinates_meas_in_scene =
            measurement_in_scene * point_meas.coordinates();
          const VectorType coordinates_scene = point_scene.coordinates();
          // ds compute merge distance in geometry
          const float point_distance_in_scene =
            (coordinates_meas_in_scene - coordinates_scene).squaredNorm();
          // ds check if the geometric distance criteria is satisfied
          if (point_distance_in_scene < maximum_distance_geometry) {
            // ds copy all fields - invalidating memory of old point
            point_scene = point_meas;
            // ds merge points according to current policy TODO UACK
            // ds updating the coordinates of the moved point
            point_scene.coordinates() = (coordinates_meas_in_scene + coordinates_scene) / 2;
            merged_points_moving.insert(correspondence.moving_idx);
          }
        }
      }
      const size_t number_of_merged_points = merged_points_moving.size();
      assert(number_of_merged_points <= number_of_points_meas);
      if (number_of_merged_points == 0) {
        std::cerr << "MergerCorrespondenceHomo::compute|WARNING: all merge attempts failed"
                  << std::endl;
      }
      const float merge_ratio =
        static_cast<float>(number_of_merged_points) / ThisType::_correspondences->size();
      if (merge_ratio < 0.5) {
        std::cerr << "MergerCorrespondenceHomo::compute|WARNING: low merge ratio: " << merge_ratio
                  << " (# correspondences: " << ThisType::_correspondences->size() << ")"
                  << std::endl;
      }

      // ds if merge target was not reached we have to add new points
      if (number_of_merged_points < ThisType::param_target_number_of_merges.value()) {
        const size_t number_of_points_to_add = number_of_points_meas - number_of_merged_points;
        std::cerr << "MergerCorrespondenceHomo|adding new points: " << number_of_points_to_add
                  << std::endl;

        // ds allocate space for point addition
        ThisType::_scene->reserve(number_of_points_scene + number_of_points_to_add);

        // ds for the remaining points that have not been merged - we add them to the scene
        for (size_t index = 0; index < number_of_points_meas; ++index) {
          // ds ignore already added points
          if (merged_points_moving.count(index)) {
            continue;
          }

          // ds transform and add new point to scene
          assert(index < number_of_points_meas);
          PointType point_meas = (*ThisType::_measurement)[index];
          if (point_meas.status == Valid) {
            point_meas.template transformInPlace<Isometry, EstimateType>(measurement_in_scene);
            ThisType::_scene->emplace_back(point_meas);
          }
        }
      }
    }

    // ds toggle to avoid redundant re-computes
    ThisType::_scene_changed_flag           = false;
    ThisType::_meas_changed_flag            = false;
    ThisType::_correspondences_changed_flag = false;
    ThisType::_meas_in_scene_changed_flag   = false;
    ThisType::_status                       = MergerBase::Success;
  }

  //  SPECIALIZE(Isometry2f, PointNormal2fVectorCloud);
  //  SPECIALIZE(Isometry2f, PointIntensityDescriptor2fVectorCloud);
  //  SPECIALIZE(Isometry3f, PointIntensityDescriptor3fVectorCloud);

} // namespace srrg2_slam_interfaces

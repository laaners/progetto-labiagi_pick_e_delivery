#pragma once

#include "initializer.h"
#include <srrg_data_structures/platform.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_pcl/point_projector.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief initializer for cameras parameters
   * It attaches to topics and waits for all the parameters needed (CameraInfoMessages)
   */
  template <int GeometryDim_>
  class InitializerCamera_ : public Initializer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int GeometryDim = GeometryDim_;

    using ThisType          = InitializerCamera_<GeometryDim>;
    using BaseType          = Initializer;
    using ProjectoryType    = srrg2_core::CameraMatrixOwner_<GeometryDim>;
    using ProjectoryTypePtr = std::shared_ptr<ProjectoryType>;

    PARAM(srrg2_core::PropertyString, topic, "topic for the camera info", "/camera_info", nullptr);
    PARAM_VECTOR(srrg2_core::PropertyConfigurableVector_<ProjectoryType>,
                 camera_matrix_owners,
                 "components that holds a camera matrix",
                 nullptr);

    virtual ~InitializerCamera_() = default;

    inline virtual void initialize() override {
      using namespace srrg2_core;
      if (_initialized) {
        return;
      }

      assert(!param_topic.value().empty() && "topic string is empty");
      if (param_camera_matrix_owners.empty()) {
        throw std::runtime_error("InitializerCamera | no modules set");
      }
      if (!BaseType::_message) {
        throw std::runtime_error("InitializerCamera | no measurement");
      }

      MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(BaseType::_message);
      if (!message_pack) {
        // it's a single message
        if (BaseType::_message->topic.value() == param_topic.value()) {
          if (!_initializeCamera(BaseType::_message)) {
            return;
          }
        } else {
          return;
        }
      } else {
        const std::string& topic = param_topic.value();
        auto it                  = std::find_if(
          message_pack->messages.begin(),
          message_pack->messages.end(),
          [&topic](BaseSensorMessagePtr msg) -> bool { return (msg->topic.value() == topic); });

        if (it == message_pack->messages.end()) {
          std::cerr << FG_RED(
                         "InitializerCamera_ | message pack does not contain camera info for topic "
                         << param_topic.value())
                    << std::endl;
          return;
        }
        if (!_initializeCamera(*it)) {
          return;
        }
      }

      std::cerr << "InitializerCamera_ | initialized" << std::endl;
      _initialized = true;
    }

  protected:
    /**
     * @brief parses the messages and gathers the needed info
     * @param[in] a message
     */
    inline bool _initializeCamera(const srrg2_core::BaseSensorMessagePtr msg_) {
      if (auto camera_info_msg = std::dynamic_pointer_cast<srrg2_core::CameraInfoMessage>(msg_)) {
        for (size_t i = 0; i < param_camera_matrix_owners.size(); ++i) {
          auto cmowner = param_camera_matrix_owners[i];
          cmowner->setCameraMatrix(camera_info_msg->camera_matrix.value());
          if (cmowner->canvasRows() < 1 && cmowner->canvasCols() < 1 &&
              camera_info_msg->rows.value() > 0 && camera_info_msg->cols.value() > 0) {
            cmowner->setCanvasRows(camera_info_msg->rows.value());
            cmowner->setCanvasCols(camera_info_msg->cols.value());
          }
        }
        return true;
      }
      return false;
    }
  };

  // TODO enable this when we will be able to get Matrix2 from ros msg
  //  using InitializerCamera2D = InitializerCamera_<2>;
  using InitializerCamera3D    = InitializerCamera_<3>;
  using InitializerCamera3DPtr = std::shared_ptr<InitializerCamera3D>;

  template <int GeometryDim_>
  class InitializerStereoCamera_ : public Initializer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int GeometryDim = GeometryDim_;

    using ThisType          = InitializerStereoCamera_<GeometryDim>;
    using BaseType          = Initializer;
    using ProjectoryType    = srrg2_core::CameraMatrixOwner_<GeometryDim>;
    using ProjectoryTypePtr = std::shared_ptr<ProjectoryType>;

    PARAM(srrg2_core::PropertyString,
          topic_camera_left,
          "topic for the camera left info",
          "/camera_info",
          nullptr);
    PARAM(srrg2_core::PropertyString,
          topic_camera_right,
          "topic for the camera right info",
          "/camera_info",
          nullptr);
    PARAM_VECTOR(srrg2_core::PropertyConfigurableVector_<ProjectoryType>,
                 camera_matrix_owners_left,
                 "left components that holds a camera matrix",
                 nullptr);
    PARAM_VECTOR(srrg2_core::PropertyConfigurableVector_<ProjectoryType>,
                 camera_matrix_owners_right,
                 "right components that holds a camera matrix",
                 nullptr);

    //    PARAM(srrg2_core::PropertyString, topic_tf, "transform tree topic", "/tf", nullptr);

    virtual ~InitializerStereoCamera_() = default;

    inline virtual void initialize() override {
      using namespace srrg2_core;
      if (_initialized) {
        return;
      }

      assert(!param_topic_camera_left.value().empty() && "topic_camera_left string is empty");
      assert(!param_topic_camera_right.value().empty() && "topic_camera_right string is empty");
      if (param_camera_matrix_owners_left.empty()) {
        throw std::runtime_error(
          "InitializerStereoCamera::initialize|no modules set for camera left");
      }
      if (param_camera_matrix_owners_right.empty()) {
        throw std::runtime_error(
          "InitializerStereoCamera::initialize|no modules set for camera right");
      }
      if (!BaseType::_message) {
        throw std::runtime_error("InitializerStereoCamera::initialize|no measurement");
      }

      MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(BaseType::_message);
      if (!message_pack) {
        return;
      } else {
        const std::string& topic_left  = param_topic_camera_left.value();
        const std::string& topic_right = param_topic_camera_right.value();
        //        const std::string& topic_tf    = param_topic_tf.value();

        auto it_left  = std::find_if(message_pack->messages.begin(),
                                    message_pack->messages.end(),
                                    [&topic_left](BaseSensorMessagePtr msg) -> bool {
                                      return (msg->topic.value() == topic_left);
                                    });
        auto it_right = std::find_if(message_pack->messages.begin(),
                                     message_pack->messages.end(),
                                     [&topic_right](BaseSensorMessagePtr msg) -> bool {
                                       return (msg->topic.value() == topic_right);
                                     });
        //        auto it_tf    = std::find_if(message_pack->messages.begin(),
        //                                  message_pack->messages.end(),
        //                                  [&topic_tf](BaseSensorMessagePtr msg) -> bool {
        //                                    return (msg->topic.value() == topic_tf);
        //                                  });

        if (it_left == message_pack->messages.end()) {
          std::cerr << FG_RED(
                         "InitializerStereoCamera::initialize|message pack does not contain camera "
                         "info for topic "
                         << topic_left)
                    << std::endl;
          return;
        }

        if (it_right == message_pack->messages.end()) {
          std::cerr << FG_RED(
                         "InitializerStereoCamera::initialize|message pack does not contain camera "
                         "info for topic "
                         << topic_right)
                    << std::endl;
          return;
        }

        if (!_initializeStereoCamera(*it_left, param_camera_matrix_owners_left) ||
            !_initializeStereoCamera(*it_right, param_camera_matrix_owners_right)) {
          return;
        }

        //        _finalizeStereoCamera(*it_tf, *it_left, *it_right);
      }

      std::cerr << "InitializerStereoCamera::initialize|initialized" << std::endl;
      _initialized = true;
    }

  protected:
    /**
     * @brief parses the messages and gathers the needed info
     * @param[in] a camera info message
     * @param[in] an image message
     * @param[in] a vector of camera owners where to set the retrieved camera matrix
     */
    inline bool _initializeStereoCamera(
      const srrg2_core::BaseSensorMessagePtr msg_,
      const srrg2_core::PropertyConfigurableVector_<ProjectoryType>& camera_owners_) {
      assert(msg_);
      //      assert(image_message_);
      if (auto camera_info_msg = std::dynamic_pointer_cast<srrg2_core::CameraInfoMessage>(msg_)) {
        for (size_t i = 0; i < camera_owners_.size(); ++i) {
          auto cmowner = camera_owners_[i];
          cmowner->setCameraMatrix(camera_info_msg->camera_matrix.value());

          // ds overwrite image dimensions TODO purge completely from configuration
          cmowner->setCanvasRows(camera_info_msg->rows.value());
          cmowner->setCanvasCols(camera_info_msg->cols.value());

          //          if (cmowner->canvasRows() == 0 || cmowner->canvasCols() == 0) {
          //            cmowner->setCanvasRows(image_message_->image_rows.value());
          //            cmowner->setCanvasCols(image_message_->image_cols.value());
          //          }
          if (cmowner->canvasRows() <= 0) {
            throw std::runtime_error(
              "InitializerStereoCamera_::_initializeStereoCamera|unable to retrieve canvas rows");
          }
          if (cmowner->canvasCols() <= 0) {
            throw std::runtime_error(
              "InitializerStereoCamera_::_initializeStereoCamera|unable to retrieve canvas cols");
          }
        }
        return true;
      }
      return false;
    }

    //    inline void _finalizeStereoCamera(const srrg2_core::BaseSensorMessagePtr msg_tf_,
    //                                      const srrg2_core::BaseSensorMessagePtr msg_left_,
    //                                      const srrg2_core::BaseSensorMessagePtr msg_right_
    //
    //    ) {
    //      assert(this->_platform &&
    //             "InitializerStereoCamera_::_finalizeStereoCamera| platform not set");
    //      this->_platform->add(msg_tf_);
    //      //      Isometry3f camera_right_in_left;
    //      //      this->_platform->getTransform(
    //      //        camera_right_in_left, msg_right_->frame_id.value(),
    //      msg_left_->frame_id.value());
    //      //      std::cerr << camera_right_in_left.translation().transpose() << std::endl;
    //    }
  };

  using InitializerStereoCamera3D    = InitializerStereoCamera_<3>;
  using InitializerStereoCamera3DPtr = std::shared_ptr<InitializerStereoCamera3D>;

} // namespace srrg2_slam_interfaces

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include <a2w_fastlio_msgs/msg/global_tf_owner.hpp>
#include <a2w_fastlio_msgs/msg/localization_status.hpp>
#include <a2w_fastlio_msgs/msg/registration_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"
#include "a2w_fastlio_common/global_tf_ownership.hpp"
#include "a2w_fastlio_localization/fine_failure_capture.hpp"
#include "a2w_fastlio_localization/local_map_selector.hpp"
#include "a2w_fastlio_localization/fine_match_diagnostics.hpp"
#include "a2w_fastlio_localization/global_relocalizer.hpp"
#include "a2w_fastlio_localization/localization_manager.hpp"
#include "a2w_fastlio_localization/localization_monitor.hpp"
#include "a2w_fastlio_localization/map_matcher.hpp"
#include "a2w_fastlio_localization/normal_matching_worker.hpp"
#include "a2w_fastlio_localization/relocalization_worker.hpp"
#include "a2w_fastlio_map/descriptor_database.hpp"
#include "a2w_fastlio_map/keyframe_database.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"

namespace a2w_fastlio_localization
{
namespace
{

std::int64_t nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

std::int64_t steadyNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

builtin_interfaces::msg::Time stampMessage(const std::int64_t value)
{
  return static_cast<builtin_interfaces::msg::Time>(rclcpp::Time{value});
}

geometry_msgs::msg::Pose poseMessage(const a2w_fastlio_common::Pose3d & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation.x();
  result.position.y = pose.translation.y();
  result.position.z = pose.translation.z();
  result.orientation.w = pose.rotation.w();
  result.orientation.x = pose.rotation.x();
  result.orientation.y = pose.rotation.y();
  result.orientation.z = pose.rotation.z();
  return result;
}

a2w_fastlio_common::Pose3d compose(
  const a2w_fastlio_common::Pose3d & lhs, const a2w_fastlio_common::Pose3d & rhs)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = (lhs.rotation.normalized() * rhs.rotation.normalized()).normalized();
  result.translation = lhs.rotation.normalized() * rhs.translation + lhs.translation;
  return result;
}

a2w_fastlio_common::Pose3d inverse(const a2w_fastlio_common::Pose3d & pose)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = pose.rotation.normalized().conjugate();
  result.translation = -(result.rotation * pose.translation);
  return result;
}

struct AdaptedFrame
{
  bool success{false};
  std::string reason;
  a2w_fastlio_common::FrontendFrame frame;
};

AdaptedFrame adapt(
  const nav_msgs::msg::Odometry & odometry,
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & odom_frame,
  const std::string & tracking_frame)
{
  AdaptedFrame result;
  if (nanoseconds(odometry.header.stamp) != nanoseconds(cloud.header.stamp)) {
    result.reason = "timestamp_mismatch";
    return result;
  }
  if (odometry.header.frame_id != odom_frame || odometry.child_frame_id != tracking_frame ||
    cloud.header.frame_id != tracking_frame)
  {
    result.reason = "frame_contract_mismatch";
    return result;
  }
  const auto & p = odometry.pose.pose.position;
  const auto & q = odometry.pose.pose.orientation;
  result.frame.stamp_ns = nanoseconds(odometry.header.stamp);
  result.frame.odom_pose.translation = {p.x, p.y, p.z};
  result.frame.odom_pose.rotation = {q.w, q.x, q.y, q.z};
  if (!a2w_fastlio_common::isFinitePose(result.frame.odom_pose) ||
    result.frame.odom_pose.rotation.squaredNorm() <= 1.0e-12)
  {
    result.reason = "invalid_odometry_pose";
    return result;
  }
  result.frame.odom_pose.rotation.normalize();
  auto converted = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  pcl::fromROSMsg(cloud, *converted);
  if (converted->empty()) {
    result.reason = "empty_body_cloud";
    return result;
  }
  result.frame.body_cloud = converted;
  result.success = true;
  return result;
}

}  // namespace

class LocalizationNode final : public rclcpp::Node
{
public:
  LocalizationNode() : Node("localization_node")
  {
    const auto bundle_path = declare_parameter<std::string>("map_bundle_path", "");
    if (bundle_path.empty() || !std::filesystem::path{bundle_path}.is_absolute()) {
      throw std::invalid_argument{"map_bundle_path must be a non-empty absolute path"};
    }
    bundle_ = a2w_fastlio_map::MapBundleReader{}.read(bundle_path);
    if (bundle_.mode != a2w_fastlio_map::ReadMode::kReadOnly) {
      throw std::runtime_error{"Localization requires a read-only Map Bundle"};
    }
    map_frame_ = declare_parameter<std::string>("map_frame", bundle_.data.metadata.map_frame);
    odom_frame_ = declare_parameter<std::string>("odom_frame", bundle_.data.metadata.odom_frame);
    tracking_frame_ = declare_parameter<std::string>(
      "tracking_frame", bundle_.data.metadata.tracking_frame);
    if (map_frame_ != bundle_.data.metadata.map_frame ||
      odom_frame_ != bundle_.data.metadata.odom_frame ||
      tracking_frame_ != bundle_.data.metadata.tracking_frame)
    {
      throw std::invalid_argument{"configured frames differ from Map Bundle metadata"};
    }

    a2w_fastlio_map::MapSnapshot snapshot;
    snapshot.keyframes = bundle_.data.keyframes;
    snapshot.descriptors = bundle_.data.descriptors;
    snapshot.global_map = bundle_.data.global_map;
    algorithm_config_ = algorithmConfig();
    const auto algorithms = a2w_fastlio_common::createDefaultAlgorithmSuite(algorithm_config_);
    const auto local_map_config = localMapConfig();
    selector_ = std::make_shared<LocalMapSelector>(snapshot, selectorConfig());
    matcher_ = std::make_shared<MapMatcher>(snapshot, algorithms.registration, local_map_config);
    handoff_ab_enabled_ = declare_parameter<bool>("diagnostics.handoff_ab", false);
    const auto capture_enabled = declare_parameter<bool>(
      "diagnostics.capture_first_fine_failure", false);
    const auto ros_log_dir = std::getenv("ROS_LOG_DIR");
    const auto default_capture_root = ros_log_dir ?
      (std::filesystem::path{ros_log_dir} / "fine_failure_captures").string() : std::string{};
    const auto capture_root = declare_parameter<std::string>(
      "diagnostics.failure_capture_directory", default_capture_root);
    fine_failure_capture_ = std::make_unique<FineFailureCapture>(FineFailureCaptureConfig{
        capture_enabled, std::filesystem::path{capture_root}});
    manager_ = std::make_unique<LocalizationManager>(managerConfig(),
      [this](const auto & frame, const auto & predicted, const auto mode) {
        const auto selection = selector_->select(predicted);
        if (mode == MapMatchingMode::kFineOnly) {
          pending_fine_selection_ = captureFineSelection(
            frame.stamp_ns, now().nanoseconds() - frame.stamp_ns,
            selection, bundle_.data.keyframes);
        }
        if (handoff_ab_enabled_ && mode == MapMatchingMode::kFineOnly &&
          frame.stamp_ns > handoff_after_stamp_ns_.load() &&
          handoff_ab_pending_.exchange(false))
        {
          return diagnoseHandoff(frame, selection);
        }
        return matcher_->match(frame, selection, mode);
      });
    const auto monitor_config = monitorConfig();
    monitor_ = std::make_unique<LocalizationMonitor>(monitor_config);
    normal_match_gate_ = std::make_unique<NormalMatchMotionGate>(normalMatchGateConfig());
    normal_matching_worker_ = std::make_unique<NormalMatchingWorker>(
      [this](const auto & frame) {
        const auto status = monitor_->latest();
        const auto mode = status.state == LocalizationState::kLocalized &&
          !status.full_recovery_required ?
          MapMatchingMode::kFineOnly : MapMatchingMode::kFull;
        return manager_->matchAndUpdate(frame, mode);
      },
      [this](const auto & result) {
        std::lock_guard<std::mutex> lock{result_mutex_};
        pending_normal_result_ = result;
      });
    const auto normal_match_timer_ms = declare_parameter<int>("match_interval_ms", 1000);
    if (normal_match_timer_ms <= 0) {
      throw std::invalid_argument{"match_interval_ms must be positive"};
    }
    global_relocalization_enabled_ = declare_parameter<bool>(
      "global_relocalization.enabled", true);
    if (global_relocalization_enabled_) {
      relocalization_keyframes_ = std::make_shared<a2w_fastlio_map::KeyFrameDatabase>();
      relocalization_descriptors_ = std::make_shared<a2w_fastlio_map::DescriptorDatabase>();
      for (const auto & keyframe : bundle_.data.keyframes) {
        if (!relocalization_keyframes_->add(keyframe)) {
          throw std::runtime_error{"Map Bundle has invalid relocalization keyframes"};
        }
      }
      for (const auto & descriptor : bundle_.data.descriptors) {
        relocalization_descriptors_->add(descriptor.keyframe_id, descriptor.descriptor);
      }
      place_recognition_ = algorithms.place_recognition;
      relocalization_config_ = globalRelocalizationConfig();
      relocalizer_ = std::make_shared<GlobalRelocalizer>(
        relocalization_descriptors_, relocalization_keyframes_, algorithms.registration,
        local_map_config);
      const auto confirmation_config = sessionConfig(monitor_config.relocalization_timeout_ns);
      relocalization_session_ = std::make_unique<RelocalizationSession>(confirmation_config);
      relocalization_config_.maximum_translation_difference_m =
        confirmation_config.maximum_translation_difference_m;
      relocalization_config_.maximum_rotation_difference_rad =
        confirmation_config.maximum_rotation_difference_rad;
      initialization_relocalization_config_ = initializationRelocalizationConfig(
        relocalization_config_);
      const auto queue_capacity = declare_parameter<int>(
        "global_relocalization.worker_queue_capacity", 1);
      const auto time_budget_ms = declare_parameter<int>(
        "global_relocalization.evaluation_time_budget_ms", 5000);
      if (queue_capacity <= 0 || time_budget_ms <= 0) {
        throw std::invalid_argument{"global relocalization worker settings must be positive"};
      }
      relocalization_time_budget_ms_ = static_cast<double>(time_budget_ms);
      relocalization_worker_ = std::make_unique<RelocalizationWorker>(
        [this](const auto & frame) {return evaluateRelocalization(frame);},
        [this](const auto & result) {
          std::lock_guard<std::mutex> lock{result_mutex_};
          if (pending_relocalization_results_.size() == 8U) {
            pending_relocalization_results_.pop_front();
          }
          pending_relocalization_results_.push_back(result);
        },
        static_cast<std::size_t>(queue_capacity));
    }

    const auto input_qos = qos("input", 20, false);
    const auto output_qos = qos("output", 20, false);
    const auto path_qos = qos("path", 1, true);
    const auto owner_qos = qos("owner", 10, true);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/Odometry");
    const auto cloud_topic = declare_parameter<std::string>(
      "body_cloud_topic", "/cloud_registered_body");
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("pose_topic", "/localization/pose"), output_qos);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("localized_odom_topic", "/localization/odom"), output_qos);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter<std::string>("path_topic", "/localization/path"), path_qos);
    status_publisher_ = create_publisher<a2w_fastlio_msgs::msg::RegistrationStatus>(
      declare_parameter<std::string>("registration_status_topic", "/localization/registration_status"),
      output_qos);
    monitor_status_publisher_ = create_publisher<a2w_fastlio_msgs::msg::LocalizationStatus>(
      declare_parameter<std::string>("localization_status_topic", "/localization/status"),
      output_qos);
    owner_topic_ = declare_parameter<std::string>(
      "global_tf_owner_topic", "/a2w_fastlio/global_tf_owner");
    owner_publisher_ = create_publisher<a2w_fastlio_msgs::msg::GlobalTfOwner>(owner_topic_, owner_qos);
    owner_subscription_ = create_subscription<a2w_fastlio_msgs::msg::GlobalTfOwner>(
      owner_topic_, owner_qos,
      std::bind(&LocalizationNode::observeOwner, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto queue = declare_parameter<int>("sync_queue_size", 20);
    if (queue <= 0) {
      throw std::invalid_argument{"sync_queue_size must be positive"};
    }
    odometry_subscriber_.subscribe(this, odom_topic, input_qos.get_rmw_qos_profile());
    cloud_subscriber_.subscribe(this, cloud_topic, input_qos.get_rmw_qos_profile());
    synchronizer_ = std::make_shared<Synchronizer>(
      odometry_subscriber_, cloud_subscriber_, static_cast<std::uint32_t>(queue));
    synchronizer_->registerCallback(std::bind(
      &LocalizationNode::inputCallback, this, std::placeholders::_1, std::placeholders::_2));

    owner_id_ = declare_parameter<std::string>("owner_id", "localization_backend");
    const auto conflict_ms = declare_parameter<int>("owner_conflict_window_ms", 1000);
    const auto heartbeat_ms = declare_parameter<int>("owner_heartbeat_ms", 250);
    const auto tf_period_ms = declare_parameter<int>("global_tf_publish_period_ms", 50);
    if (owner_id_.empty() || conflict_ms <= 0 || heartbeat_ms <= 0 || tf_period_ms <= 0) {
      throw std::invalid_argument{"invalid global TF owner configuration"};
    }
    owner_state_ = std::make_unique<a2w_fastlio_common::GlobalTfOwnerState>(
      a2w_fastlio_common::GlobalTfOwnerConfig{
        owner_id_, "localization", map_frame_, odom_frame_,
        static_cast<std::int64_t>(conflict_ms) * 1'000'000LL});
    owner_state_->start(now().nanoseconds());
    owner_timer_ = create_wall_timer(
      std::chrono::milliseconds{heartbeat_ms}, std::bind(&LocalizationNode::heartbeat, this));
    global_tf_timer_ = create_wall_timer(
      std::chrono::milliseconds{tf_period_ms},
      std::bind(&LocalizationNode::publishGlobalTransform, this));
    normal_match_timer_ = create_wall_timer(
      std::chrono::milliseconds{normal_match_timer_ms},
      std::bind(&LocalizationNode::normalMatchingTimer, this));
    const auto input_timeout_ms = declare_parameter<int>("input_timeout_ms", 1000);
    if (input_timeout_ms <= 0) {
      throw std::invalid_argument{"input_timeout_ms must be positive"};
    }
    input_timeout_ns_ = static_cast<std::int64_t>(input_timeout_ms) * 1'000'000LL;
    result_timer_ = create_wall_timer(std::chrono::milliseconds{50},
      std::bind(&LocalizationNode::resultAndHealthTimer, this));
    const auto max_path_poses = declare_parameter<int>("max_path_poses", 10000);
    if (max_path_poses <= 0) {
      throw std::invalid_argument{"max_path_poses must be positive"};
    }
    max_path_poses_ = static_cast<std::size_t>(max_path_poses);
    RCLCPP_INFO(get_logger(), "Loaded read-only Map Bundle %s", bundle_.root.c_str());
  }

  ~LocalizationNode() override
  {
    normal_match_timer_.reset();
    result_timer_.reset();
    normal_matching_worker_.reset();
    relocalization_worker_.reset();
  }

private:
  using Synchronizer = message_filters::TimeSynchronizer<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  rclcpp::QoS qos(const std::string & prefix, int depth_default, bool transient)
  {
    const auto depth = declare_parameter<int>(prefix + "_qos_depth", depth_default);
    const auto reliability = declare_parameter<std::string>(
      prefix + "_qos_reliability", "reliable");
    const auto durability = declare_parameter<std::string>(
      prefix + "_qos_durability", transient ? "transient_local" : "volatile");
    if (depth <= 0 || (reliability != "reliable" && reliability != "best_effort") ||
      (durability != "volatile" && durability != "transient_local"))
    {
      throw std::invalid_argument{"invalid QoS configuration for " + prefix};
    }
    auto result = rclcpp::QoS{rclcpp::KeepLast{static_cast<std::size_t>(depth)}};
    reliability == "reliable" ? result.reliable() : result.best_effort();
    durability == "transient_local" ? result.transient_local() : result.durability_volatile();
    return result;
  }

  a2w_fastlio_common::DefaultAlgorithmSuiteConfig algorithmConfig()
  {
    a2w_fastlio_common::DefaultAlgorithmSuiteConfig config;
    const auto rings = declare_parameter<int>(
      "scan_context.rings", static_cast<int>(config.place_recognition.rings));
    const auto sectors = declare_parameter<int>(
      "scan_context.sectors", static_cast<int>(config.place_recognition.sectors));
    config.place_recognition.max_radius_m = declare_parameter<double>(
      "scan_context.max_radius_m", config.place_recognition.max_radius_m);
    config.place_recognition.sensor_height_m = declare_parameter<double>(
      "scan_context.sensor_height_m", config.place_recognition.sensor_height_m);
    if (rings <= 0 || sectors <= 0) {
      throw std::invalid_argument{"scan-context dimensions must be positive"};
    }
    config.place_recognition.rings = static_cast<std::size_t>(rings);
    config.place_recognition.sectors = static_cast<std::size_t>(sectors);
    config.coarse.normal_radius_m = declare_parameter<double>(
      "quatro.fpfh_normal_radius_m", config.coarse.normal_radius_m);
    config.coarse.feature_radius_m = declare_parameter<double>(
      "quatro.fpfh_radius_m", config.coarse.feature_radius_m);
    config.coarse.noise_bound_m = declare_parameter<double>(
      "quatro.noise_bound_m", config.coarse.noise_bound_m);
    config.coarse.rotation_gnc_factor = declare_parameter<double>(
      "quatro.rotation_gnc_factor", config.coarse.rotation_gnc_factor);
    config.coarse.rotation_cost_threshold = declare_parameter<double>(
      "quatro.rotation_cost_threshold", config.coarse.rotation_cost_threshold);
    config.coarse.rotation_max_iterations = declare_parameter<int>(
      "quatro.rotation_max_iterations", config.coarse.rotation_max_iterations);
    config.coarse.estimate_scale = declare_parameter<bool>(
      "quatro.estimate_scale", config.coarse.estimate_scale);
    config.coarse.optimized_matching = declare_parameter<bool>(
      "quatro.optimized_matching", config.coarse.optimized_matching);
    config.coarse.descriptor_distance_threshold = declare_parameter<double>(
      "quatro.descriptor_distance_threshold", config.coarse.descriptor_distance_threshold);
    config.coarse.maximum_correspondences = declare_parameter<int>(
      "quatro.maximum_correspondences", config.coarse.maximum_correspondences);
    const auto coarse_minimum = declare_parameter<int>(
      "quatro.minimum_points", static_cast<int>(config.coarse.minimum_points));
    config.fine.thread_count = declare_parameter<int>(
      "nano_gicp.thread_count", config.fine.thread_count);
    config.fine.correspondence_randomness = declare_parameter<int>(
      "nano_gicp.correspondence_randomness", config.fine.correspondence_randomness);
    config.fine.maximum_iterations = declare_parameter<int>(
      "nano_gicp.maximum_iterations", config.fine.maximum_iterations);
    config.fine.transformation_epsilon = declare_parameter<double>(
      "nano_gicp.transformation_epsilon", config.fine.transformation_epsilon);
    config.fine.rotation_epsilon = declare_parameter<double>(
      "nano_gicp.rotation_epsilon", config.fine.rotation_epsilon);
    config.fine.regularization_method = declare_parameter<int>(
      "nano_gicp.regularization_method", config.fine.regularization_method);
    config.fine.fitness_score_max_range_m = declare_parameter<double>(
      "nano_gicp.fitness_score_max_range_m", config.fine.fitness_score_max_range_m);
    config.fine.source_voxel_leaf_m = declare_parameter<double>(
      "nano_gicp.source_voxel_leaf_m", config.fine.source_voxel_leaf_m);
    const auto fine_minimum = declare_parameter<int>(
      "nano_gicp.minimum_points", static_cast<int>(config.fine.minimum_points));
    config.fine.maximum_correspondence_distance_m = declare_parameter<double>(
      "nano_gicp.maximum_correspondence_distance_m",
      config.fine.maximum_correspondence_distance_m);
    config.validation.maximum_fitness = declare_parameter<double>(
      "validation.maximum_fitness", config.validation.maximum_fitness);
    config.validation.minimum_overlap = declare_parameter<double>(
      "validation.minimum_overlap", config.validation.minimum_overlap);
    const auto minimum_correspondences = declare_parameter<int>(
      "validation.minimum_correspondences",
      static_cast<int>(config.validation.minimum_correspondences));
    config.validation.maximum_translation_jump_m = declare_parameter<double>(
      "validation.maximum_translation_jump_m", config.validation.maximum_translation_jump_m);
    config.validation.maximum_rotation_jump_rad = declare_parameter<double>(
      "validation.maximum_rotation_jump_rad", config.validation.maximum_rotation_jump_rad);
    config.validation.minimum_candidate_distance_separation = declare_parameter<double>(
      "validation.minimum_candidate_distance_separation",
      config.validation.minimum_candidate_distance_separation);
    config.evidence_distance_m = declare_parameter<double>(
      "pipeline.evidence_distance_m", config.evidence_distance_m);
    if (coarse_minimum <= 0 || fine_minimum <= 0 || minimum_correspondences <= 0) {
      throw std::invalid_argument{"registration point/count thresholds must be positive"};
    }
    config.coarse.minimum_points = static_cast<std::size_t>(coarse_minimum);
    config.fine.minimum_points = static_cast<std::size_t>(fine_minimum);
    config.validation.minimum_correspondences =
      static_cast<std::size_t>(minimum_correspondences);
    return config;
  }

  LocalMapSelectorConfig selectorConfig()
  {
    LocalMapSelectorConfig config;
    config.radius_m = declare_parameter<double>("local_map.selection_radius_m", config.radius_m);
    const auto maximum = declare_parameter<int>(
      "local_map.max_neighbors", static_cast<int>(config.max_neighbors));
    const auto minimum = declare_parameter<int>(
      "local_map.minimum_neighbors", static_cast<int>(config.minimum_neighbors));
    if (maximum <= 0 || minimum <= 0) {
      throw std::invalid_argument{"local-map neighbor thresholds must be positive"};
    }
    config.max_neighbors = static_cast<std::size_t>(maximum);
    config.minimum_neighbors = static_cast<std::size_t>(minimum);
    return config;
  }

  a2w_fastlio_common::LocalMapConfig localMapConfig()
  {
    a2w_fastlio_common::LocalMapConfig config;
    config.voxel_leaf_m = declare_parameter<double>("local_map.voxel_leaf_m", config.voxel_leaf_m);
    const auto max_points = declare_parameter<int>(
      "local_map.max_points", static_cast<int>(config.max_points));
    if (max_points <= 0) {
      throw std::invalid_argument{"local_map.max_points must be positive"};
    }
    config.max_points = static_cast<std::size_t>(max_points);
    return config;
  }

  LocalizationManagerConfig managerConfig()
  {
    LocalizationManagerConfig config;
    config.maximum_correction_translation_jump_m = declare_parameter<double>(
      "maximum_correction_translation_jump_m", config.maximum_correction_translation_jump_m);
    config.maximum_correction_rotation_jump_rad = declare_parameter<double>(
      "maximum_correction_rotation_jump_rad", config.maximum_correction_rotation_jump_rad);
    const auto maximum_age_ms = declare_parameter<int>("maximum_match_result_age_ms", 3000);
    if (maximum_age_ms <= 0) {
      throw std::invalid_argument{"maximum_match_result_age_ms must be positive"};
    }
    config.maximum_match_result_age_ns = static_cast<std::int64_t>(maximum_age_ms) * 1'000'000LL;
    return config;
  }

  NormalMatchMotionGateConfig normalMatchGateConfig()
  {
    NormalMatchMotionGateConfig config;
    config.translation_threshold_m = declare_parameter<double>(
      "normal_match.translation_threshold_m", config.translation_threshold_m);
    const auto rotation_threshold_deg = declare_parameter<double>(
      "normal_match.rotation_threshold_deg", 5.0);
    const auto maximum_elapsed_ms = declare_parameter<int>(
      "normal_match.maximum_elapsed_ms",
      static_cast<int>(config.maximum_elapsed_ns / 1'000'000LL));
    if (!std::isfinite(rotation_threshold_deg) || rotation_threshold_deg <= 0.0 ||
      maximum_elapsed_ms <= 0)
    {
      throw std::invalid_argument{"invalid normal match motion gate parameters"};
    }
    constexpr double degrees_to_radians = 0.017453292519943295;
    config.rotation_threshold_rad = rotation_threshold_deg * degrees_to_radians;
    config.maximum_elapsed_ns = static_cast<std::int64_t>(maximum_elapsed_ms) * 1'000'000LL;
    return config;
  }

  LocalizationMonitorConfig monitorConfig()
  {
    LocalizationMonitorConfig config;
    const auto positiveCount = [this](const std::string & name, const std::size_t value) {
        const auto declared = declare_parameter<int>(name, static_cast<int>(value));
        if (declared <= 0) {
          throw std::invalid_argument{name + " must be positive"};
        }
        return static_cast<std::size_t>(declared);
      };
    config.initialization_successes_required = positiveCount(
      "monitor.initialization_successes_required", config.initialization_successes_required);
    config.degraded_failures_required = positiveCount(
      "monitor.degraded_failures_required", config.degraded_failures_required);
    config.lost_failures_required = positiveCount(
      "monitor.lost_failures_required", config.lost_failures_required);
    config.normal_recovery_successes_required = positiveCount(
      "monitor.normal_recovery_successes_required", config.normal_recovery_successes_required);
    config.relocalization_successes_required = positiveCount(
      "monitor.relocalization_successes_required", config.relocalization_successes_required);
    const auto stale_ms = declare_parameter<int>(
      "monitor.correction_stale_after_ms",
      static_cast<int>(config.correction_stale_after_ns / 1'000'000LL));
    const auto timeout_ms = declare_parameter<int>(
      "monitor.relocalization_timeout_ms",
      static_cast<int>(config.relocalization_timeout_ns / 1'000'000LL));
    if (stale_ms <= 0 || timeout_ms <= 0) {
      throw std::invalid_argument{"monitor time thresholds must be positive"};
    }
    config.correction_stale_after_ns = static_cast<std::int64_t>(stale_ms) * 1'000'000LL;
    config.relocalization_timeout_ns = static_cast<std::int64_t>(timeout_ms) * 1'000'000LL;
    config.strong_maximum_fitness = declare_parameter<double>(
      "monitor.strong_maximum_fitness", config.strong_maximum_fitness);
    config.strong_minimum_overlap = declare_parameter<double>(
      "monitor.strong_minimum_overlap", config.strong_minimum_overlap);
    config.strong_minimum_correspondences = positiveCount(
      "monitor.strong_minimum_correspondences", config.strong_minimum_correspondences);
    return config;
  }

  RelocalizationConfig globalRelocalizationConfig()
  {
    RelocalizationConfig config;
    const auto top_k = declare_parameter<int>(
      "global_relocalization.top_k", static_cast<int>(config.top_k));
    const auto before = declare_parameter<int>(
      "global_relocalization.neighbor_keyframes_before",
      static_cast<int>(config.neighbor_keyframes_before));
    const auto after = declare_parameter<int>(
      "global_relocalization.neighbor_keyframes_after",
      static_cast<int>(config.neighbor_keyframes_after));
    const auto strong_count = declare_parameter<int>(
      "global_relocalization.strong_minimum_correspondences",
      static_cast<int>(config.strong_minimum_correspondences));
    if (top_k < 2 || before < 0 || after < 0 || strong_count <= 0) {
      throw std::invalid_argument{"invalid global relocalization count settings"};
    }
    config.top_k = static_cast<std::size_t>(top_k);
    config.neighbor_keyframes_before = static_cast<std::size_t>(before);
    config.neighbor_keyframes_after = static_cast<std::size_t>(after);
    config.strong_minimum_correspondences = static_cast<std::size_t>(strong_count);
    config.maximum_descriptor_distance = declare_parameter<double>(
      "global_relocalization.maximum_descriptor_distance",
      config.maximum_descriptor_distance);
    config.minimum_score_margin = declare_parameter<double>(
      "global_relocalization.minimum_score_margin", config.minimum_score_margin);
    config.descriptor_score_weight = declare_parameter<double>(
      "global_relocalization.descriptor_score_weight", config.descriptor_score_weight);
    config.fitness_score_weight = declare_parameter<double>(
      "global_relocalization.fitness_score_weight", config.fitness_score_weight);
    config.overlap_score_weight = declare_parameter<double>(
      "global_relocalization.overlap_score_weight", config.overlap_score_weight);
    config.strong_maximum_fitness = declare_parameter<double>(
      "global_relocalization.strong_maximum_fitness", config.strong_maximum_fitness);
    config.strong_minimum_overlap = declare_parameter<double>(
      "global_relocalization.strong_minimum_overlap", config.strong_minimum_overlap);
    const auto finite_nonnegative = [](const double value) {
        return std::isfinite(value) && value >= 0.0;
      };
    if (!finite_nonnegative(config.maximum_descriptor_distance) ||
      !finite_nonnegative(config.minimum_score_margin) ||
      !finite_nonnegative(config.descriptor_score_weight) ||
      !finite_nonnegative(config.fitness_score_weight) ||
      !finite_nonnegative(config.overlap_score_weight) ||
      config.fitness_score_weight + config.overlap_score_weight <= 0.0 ||
      !finite_nonnegative(config.strong_maximum_fitness) ||
      !std::isfinite(config.strong_minimum_overlap) ||
      config.strong_minimum_overlap < 0.0 || config.strong_minimum_overlap > 1.0)
    {
      throw std::invalid_argument{"invalid global relocalization quality settings"};
    }
    return config;
  }

  RelocalizationSessionConfig sessionConfig(const std::int64_t timeout_ns)
  {
    RelocalizationSessionConfig config;
    const auto confirmations = declare_parameter<int>(
      "global_relocalization.confirmation_count", static_cast<int>(config.confirmation_count));
    if (confirmations < 2) {
      throw std::invalid_argument{"global relocalization confirmation_count must be at least two"};
    }
    config.confirmation_count = static_cast<std::size_t>(confirmations);
    config.maximum_translation_difference_m = declare_parameter<double>(
      "global_relocalization.maximum_translation_difference_m",
      config.maximum_translation_difference_m);
    config.maximum_rotation_difference_rad = declare_parameter<double>(
      "global_relocalization.maximum_rotation_difference_rad",
      config.maximum_rotation_difference_rad);
    config.timeout_ns = timeout_ns;
    return config;
  }

  RelocalizationConfig initializationRelocalizationConfig(
    const RelocalizationConfig & global_config)
  {
    auto config = global_config;
    const auto mode = declare_parameter<std::string>("initialization.mode", "global");
    if (mode == "global") {
      return config;
    }
    if (mode != "fixed_region") {
      throw std::invalid_argument{"initialization.mode must be global or fixed_region"};
    }
    config.candidate_region.enabled = true;
    config.candidate_region.center_x = declare_parameter<double>(
      "initialization.center_x", 0.0);
    config.candidate_region.center_y = declare_parameter<double>(
      "initialization.center_y", 0.0);
    config.candidate_region.size_x = declare_parameter<double>(
      "initialization.size_x", 3.0);
    config.candidate_region.size_y = declare_parameter<double>(
      "initialization.size_y", 3.0);
    const auto anchor_id = declare_parameter<std::int64_t>(
      "initialization.anchor_keyframe_id", 0);
    const auto keyframe_window = declare_parameter<int>(
      "initialization.keyframe_window", 30);
    if (!std::isfinite(config.candidate_region.center_x) ||
      !std::isfinite(config.candidate_region.center_y) ||
      !std::isfinite(config.candidate_region.size_x) ||
      !std::isfinite(config.candidate_region.size_y) ||
      config.candidate_region.size_x <= 0.0 || config.candidate_region.size_y <= 0.0 ||
      anchor_id < 0 || keyframe_window <= 0)
    {
      throw std::invalid_argument{"invalid fixed initialization region"};
    }
    config.candidate_region.anchor_keyframe_id = static_cast<std::uint64_t>(anchor_id);
    config.candidate_region.keyframe_window = static_cast<std::size_t>(keyframe_window);
    RCLCPP_INFO(
      get_logger(),
      "冷启动固定区域：center=(%.3f, %.3f) size=(%.3f, %.3f) anchor=%lu window=%zu",
      config.candidate_region.center_x, config.candidate_region.center_y,
      config.candidate_region.size_x, config.candidate_region.size_y,
      static_cast<unsigned long>(config.candidate_region.anchor_keyframe_id),
      config.candidate_region.keyframe_window);
    return config;
  }

  RelocalizationResult evaluateRelocalization(
    const a2w_fastlio_common::FrontendFrame & frame) const
  {
    const auto started = std::chrono::steady_clock::now();
    const auto & config = initialization_relocalization_active_.load() ?
      initialization_relocalization_config_ : relocalization_config_;
    auto result = relocalizer_->evaluate(
      frame.body_cloud, place_recognition_->describe(frame.body_cloud), config);
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    result.elapsed_ms = elapsed_ms;
    result.time_budget_ms = relocalization_time_budget_ms_;
    // This measures completed synchronous work, not a cancellable deadline.
    // Preserve geometry/ambiguity outcomes; timing is an independent diagnostic.
    return result;
  }

  void publishCandidateAudits(const RelocalizationWorkResult & work)
  {
    for (const auto & audit : work.result.audits) {
      publishStatus(
        work.frame.stamp_ns, audit.accepted, audit.reason,
        audit.candidate.keyframe_id, audit.registration, "global_relocalization");
    }
    if (work.result.audits.empty()) {
      publishStatus(
        work.frame.stamp_ns, false, work.result.reason, 0U, {}, "global_relocalization");
    }
  }

  void handleRelocalizationResult(const RelocalizationWorkResult & work)
  {
    if (work.generation != relocalization_worker_->generation()) {
      return;
    }
    const auto state = monitor_->latest().state;
    const bool initializing = state == LocalizationState::kInitializing;
    if (!initializing && state != LocalizationState::kRelocalizing) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock{correction_mutex_};
      if (latest_manager_correction_stamp_ns_ &&
        work.frame.stamp_ns <= *latest_manager_correction_stamp_ns_)
      {
        publishStatus(work.frame.stamp_ns, false, "stale_relocalization_result",
          work.result.candidate_id, work.result.registration, "global_relocalization");
        return;
      }
    }
    publishCandidateAudits(work);
    auto session_result = work.result;
    // Recovery and cold start both require independent consistent frames before
    // exposing a correction to navigation, even with one very good score.
    session_result.strong = false;
    if (session_result.success) {
      session_result.map_body = compose(work.result.map_body, inverse(work.frame.odom_pose));
    }
    try {
      const auto session = relocalization_session_->observe(steadyNanoseconds(), session_result);
      const bool confirmed = session.confirmed_correction.has_value();
      if (initializing) {
        RCLCPP_INFO(
          get_logger(), "%s",
          formatInitializationDiagnostic(work.result, session).c_str());
      } else if (work.result.elapsed_ms > work.result.time_budget_ms) {
        RCLCPP_WARN(get_logger(), "全局重定位耗时超出预算：%.1f / %.1f ms，保留原始结果 %s",
          work.result.elapsed_ms, work.result.time_budget_ms, work.result.reason.c_str());
      }
      if (initializing && !session.active) {
        initialization_relocalization_active_.store(false);
      }
      if (confirmed) {
        manager_->restoreCorrection(work.frame.stamp_ns, *session.confirmed_correction);
        std::lock_guard<std::mutex> lock{correction_mutex_};
        latest_correction_ = *session.confirmed_correction;
        latest_manager_correction_stamp_ns_ = work.frame.stamp_ns;
      }
      const bool session_accepted = work.result.success && (session.active || confirmed);
      const auto monitor_status = monitor_->update(MatchEvidence{
        eventStamp(), true, session_accepted, confirmed,
        confirmed ? manager_->correctionAcceptedAgeNs() : 0,
        EvidenceSource::kRelocalization, work.result.candidate_id,
        work.result.registration, true, confirmed, session.reason == "session_timeout"});
      publishMonitorStatus(monitor_status);
      tf_allowed_.store(monitor_status.state == LocalizationState::kLocalized ||
        monitor_status.state == LocalizationState::kDegraded);
      if (!confirmed || monitor_status.state != LocalizationState::kLocalized) {
        return;
      }
      initialization_relocalization_active_.store(false);
      relocalization_worker_->cancelPending();
      if (initializing && handoff_ab_enabled_) {
        handoff_after_stamp_ns_.store(latest_input_stamp_ns_.load());
        handoff_ab_pending_.store(true);
        RCLCPP_WARN(get_logger(),
          "交接 A/B 诊断已启用：等待初始化接受之后的新同步帧；B 仅记录，"
          "不回写 correction。串行诊断增加一次匹配耗时，请勿同时导航。");
      }
      RCLCPP_INFO(
        get_logger(), "Global relocalization restored candidate %lu",
        static_cast<unsigned long>(work.result.candidate_id));
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Rejected relocalization result: %s", error.what());
    }
  }

  void beginGlobalRelocalization(const std::int64_t stamp_ns)
  {
    if (!global_relocalization_enabled_ || !relocalization_worker_ ||
      monitor_->latest().state != LocalizationState::kLost)
    {
      return;
    }
    relocalization_worker_->cancelPending();
    manager_->invalidatePendingMatches();
    relocalization_session_->start(steadyNanoseconds());
    publishMonitorStatus(monitor_->beginRelocalization(eventStamp()));
  }

  bool beginInitialGlobalLocalization(const std::int64_t stamp_ns)
  {
    if (!global_relocalization_enabled_ || !relocalization_worker_ ||
      monitor_->latest().state != LocalizationState::kInitializing)
    {
      return false;
    }
    bool expected = false;
    if (!initialization_relocalization_active_.compare_exchange_strong(expected, true)) {
      return false;
    }
    relocalization_worker_->cancelPending();
    relocalization_session_->start(steadyNanoseconds());
    return true;
  }

  void normalMatchingTimer()
  {
    // Hold through busy-check and submission so completion cannot slip between
    // the inbox check and the worker check. Worker callbacks hold no worker lock.
    std::lock_guard<std::mutex> result_lock{result_mutex_};
    if (pending_normal_result_) {
      return;
    }
    const auto status = monitor_->latest();
    const auto state = status.state;
    const bool full_recovery_pending = status.full_recovery_required ||
      state == LocalizationState::kDegraded;
    if ((state == LocalizationState::kInitializing && global_relocalization_enabled_) ||
      state == LocalizationState::kLost || state == LocalizationState::kRelocalizing ||
      normal_matching_worker_->busy() || !latest_match_frame_ ||
      (!full_recovery_pending && !normal_match_gate_->shouldSubmit(*latest_match_frame_)))
    {
      return;
    }
    const auto frame = *latest_match_frame_;
    if (normal_matching_worker_->trySubmit(frame)) {
      submitted_input_steady_ns_ = latest_input_steady_ns_;
      submitted_preprocess_ms_ = latest_preprocess_ms_;
      submitted_dispatch_steady_ns_ = steadyNanoseconds();
      normal_match_gate_->recordSubmission(frame);
      latest_match_frame_.reset();
    }
  }

  std::int64_t eventStamp() const
  {
    // State events describe acceptance/health now; source stamps remain solely
    // the key for cloud/odom pairing and correction ordering.
    return std::max(now().nanoseconds(), monitor_->latest().stamp_ns + 1);
  }

  void resultAndHealthTimer()
  {
    // All result consumers run on this node's single executor. Workers perform
    // expensive registration only, avoiding concurrent TF/status/path writes.
    std::optional<NormalMatchingWorkResult> normal;
    std::deque<RelocalizationWorkResult> global;
    {
      std::lock_guard<std::mutex> lock{result_mutex_};
      normal.swap(pending_normal_result_);
      global.swap(pending_relocalization_results_);
    }
    for (const auto & result : global) {
      handleRelocalizationResult(result);
    }
    if (normal) {
      handleNormalMatchingResult(*normal);
    }
    const auto before = monitor_->latest();
    bool correction_available;
    {
      std::lock_guard<std::mutex> lock{correction_mutex_};
      correction_available = latest_correction_.has_value();
    }
    const bool input_fresh = last_input_steady_ns_ &&
      steadyNanoseconds() - *last_input_steady_ns_ < input_timeout_ns_;
    const auto status = monitor_->checkHealth(eventStamp(), correction_available,
      manager_->correctionAcceptedAgeNs(), input_fresh);
    publishMonitorStatus(status);
    tf_allowed_.store(input_fresh &&
      (status.state == LocalizationState::kLocalized || status.state == LocalizationState::kDegraded));
    if (status.state == LocalizationState::kLost && before.state != LocalizationState::kLost) {
      manager_->invalidatePendingMatches();
      latest_match_frame_.reset();
      beginGlobalRelocalization(status.stamp_ns);
    }
  }

  MapMatchResult diagnoseHandoff(
    const a2w_fastlio_common::FrontendFrame & frame, const SelectionResult & selection)
  {
    const auto accepted_age_ns = manager_->correctionAcceptedAgeNs();
    std::optional<a2w_fastlio_common::Pose3d> correction;
    std::int64_t correction_source_stamp = 0;
    {
      std::lock_guard<std::mutex> lock{correction_mutex_};
      correction = latest_correction_;
      correction_source_stamp = latest_manager_correction_stamp_ns_.value_or(0);
    }
    const auto source_age_ms = static_cast<double>(now().nanoseconds() - frame.stamp_ns) / 1e6;
    const auto comparison = matcher_->compareModes(frame, selection);
    std::ostringstream report;
    report << std::fixed << std::setprecision(6)
           << "[handoff_ab] 同帧诊断（B不回写） source_stamp_ns=" << frame.stamp_ns
           << " correction_source_stamp_ns=" << correction_source_stamp
           << " source_age_ms=" << source_age_ms
           << " correction_source_age_ms="
           << static_cast<double>(frame.stamp_ns - correction_source_stamp) / 1e6
           << " correction_accepted_age_ms=" << static_cast<double>(accepted_age_ns) / 1e6
           << " source_frame=" << tracking_frame_ << " target_frame=" << map_frame_;
    const auto pose = [&report](const char * name, const a2w_fastlio_common::Pose3d & value) {
        report << "\n" << name << " xyz=[" << value.translation.transpose()
               << "] q_xyzw=[" << value.rotation.coeffs().transpose() << "]";
      };
    pose("T_camera_init_body(source)", frame.odom_pose);
    if (correction) {
      pose("last_T_map_camera_init", *correction);
      const auto expected = compose(*correction, frame.odom_pose);
      report << "\nprediction_check translation_error_m="
             << (expected.translation - selection.initial_map_body.translation).norm()
             << " rotation_error_rad="
             << expected.rotation.angularDistance(selection.initial_map_body.rotation);
    }
    pose("fine_only_actual_initial_guess_T_map_body", selection.initial_map_body);
    report << "\nselected_keyframes=[";
    for (const auto id : selection.keyframe_ids) {report << id << " ";}
    report << "]";
    const auto outcome = [&report, &pose](
        const char * label, const MapMatchResult & value, const double latency_ms) {
        report << "\n" << label << " accepted=" << value.success
               << " converged=" << value.registration.converged
               << " match_latency_ms=" << latency_ms
               << " nano_gicp_ms=" << value.registration.elapsed_ms
               << " source_points=" << value.source_points << " target_points=" << value.target_points
               << " fitness=" << value.registration.fitness << " overlap=" << value.registration.overlap
               << " correspondences=" << value.registration.correspondence_count
               << " reason=" << value.reason;
        pose("solver_final_T_map_body", value.registration.transform);
      };
    outcome("A_fine_only", comparison.fine_only, comparison.fine_only_latency_ms);
    outcome("B_full", comparison.full, comparison.full_latency_ms);
    RCLCPP_INFO(get_logger(), "%s", report.str().c_str());
    // Preserve the existing online decision: never promote a successful B.
    return comparison.fine_only;
  }

  void handleNormalMatchingResult(const NormalMatchingWorkResult & work)
  {
    const auto consumer_started = steadyNanoseconds();
    const auto publication = normalResultPublication(work.output,
      manager_->isCurrentResult(work.output), monitor_->latest().state);
    auto registration = work.output.registration;
    const auto nano_gicp_ms = registration.elapsed_ms;
    registration.elapsed_ms = work.elapsed_ms;
    publishStatus(
      work.output.stamp_ns, publication.accepted, publication.reason,
      work.output.candidate_id, registration);
    const auto publication_ms = static_cast<double>(steadyNanoseconds() - consumer_started) / 1e6;
    const auto & timings = work.output.timings;
    const double wait_ms = std::max(0.0,
      static_cast<double>(submitted_dispatch_steady_ns_ - submitted_input_steady_ns_) / 1e6 -
      submitted_preprocess_ms_) + work.queue_wait_ms;
    const double result_wait_ms = std::max(0.0,
      static_cast<double>(consumer_started - submitted_dispatch_steady_ns_) / 1e6 -
      work.queue_wait_ms - work.elapsed_ms);
    const double residual_ms = std::max(0.0, timings.total_ms - timings.submap_ms -
      timings.coarse_ms - timings.fine_ms - timings.evidence_ms - timings.validation_ms);
    RCLCPP_INFO(get_logger(),
      "offline_match_stages source_stamp_ns=%ld wait_ms=%.3f preprocess_ms=%.3f "
      "submap_ms=%.3f coarse_ms=%.3f fine_ms=%.3f evidence_ms=%.3f validation_ms=%.3f "
      "residual_ms=%.3f worker_ms=%.3f result_wait_ms=%.3f status_publish_ms=%.3f "
      "end_to_end_status_ms=%.3f source_age_ms=%.3f cache_hit=%d manager_accepted=%d "
      "accepted=%d mode=%s reason=%s",
      static_cast<long>(work.frame.stamp_ns), wait_ms, submitted_preprocess_ms_,
      timings.submap_ms, timings.coarse_ms, timings.fine_ms, timings.evidence_ms,
      timings.validation_ms, residual_ms, work.elapsed_ms, result_wait_ms, publication_ms,
      static_cast<double>(steadyNanoseconds() - submitted_input_steady_ns_) / 1e6,
      static_cast<double>(now().nanoseconds() - work.frame.stamp_ns) / 1e6,
      timings.target_cache_hit, work.output.match_accepted, publication.accepted,
      work.output.matching_mode == MapMatchingMode::kFineOnly ? "fine_only" : "full",
      publication.reason.c_str());

    const auto correction_age_ns = manager_->correctionAcceptedAgeNs();
    // Detailed online diagnostics are failure-triggered. Topic metrics remain
    // unchanged; the old per-match console metrics are available at DEBUG.
    RCLCPP_DEBUG(
      get_logger(),
      "normal_match_metrics latency_ms=%.3f nano_gicp_ms=%.3f fitness=%.6f "
      "overlap=%.6f correspondences=%zu source_points=%zu target_points=%zu "
      "correction_accepted_age_ms=%.3f mode=%s accepted=%s reason=%s",
      work.elapsed_ms, nano_gicp_ms, work.output.registration.fitness,
      work.output.registration.overlap, work.output.registration.correspondence_count,
      work.output.source_points, work.output.target_points,
      static_cast<double>(correction_age_ns) / 1.0e6,
      work.output.matching_mode == MapMatchingMode::kFineOnly ? "fine_only" : "full",
      work.output.match_accepted ? "true" : "false", work.output.reason.c_str());

    if (work.output.matching_mode == MapMatchingMode::kFineOnly) {
      FineMatchRecord diagnostic;
      diagnostic.output = work.output;
      if (!publication.usable) {
        diagnostic.output.result_stale = true;
        diagnostic.output.match_accepted = false;
      }
      diagnostic.selection = pending_fine_selection_;
      diagnostic.odom_pose = work.frame.odom_pose;
      diagnostic.source_cloud = work.frame.body_cloud;
      diagnostic.source_age_ns = now().nanoseconds() - work.frame.stamp_ns;
      diagnostic.correction_accepted_age_ns = correction_age_ns;
      const auto prediction_accepted = work.output.prediction_correction_accepted_steady_ns;
      diagnostic.prediction_correction_accepted_age_ns = prediction_accepted >= 0 ?
        std::max<std::int64_t>(0, steadyNanoseconds() - prediction_accepted) : -1;
      diagnostic.match_latency_ms = work.elapsed_ms;
      const auto summary = fine_match_diagnostics_.observe(diagnostic);
      if (summary) {
        RCLCPP_WARN(get_logger(), "%s", summary->c_str());
      }
      const auto capture = fine_failure_capture_->observe(diagnostic, algorithm_config_);
      if (capture) {
        if (capture->captured) {
          RCLCPP_WARN(
            get_logger(), "已冻结首个 fine-only 失败帧: %s",
            capture->directory.c_str());
        } else {
          RCLCPP_WARN(
            get_logger(), "fine-only 失败帧冻结失败: %s",
            capture->reason.c_str());
        }
      }
    }

    if (!publication.usable) {
      if (work.output.matching_mode != MapMatchingMode::kFineOnly) {
        RCLCPP_WARN(get_logger(), "Discarded stale normal match result for stamp %ld",
          static_cast<long>(work.frame.stamp_ns));
      }
      return;
    }
    const auto current_state = monitor_->latest().state;
    if (current_state == LocalizationState::kLost ||
      current_state == LocalizationState::kRelocalizing)
    {
      return;
    }
    if (work.output.valid) {
      std::lock_guard<std::mutex> lock{correction_mutex_};
      latest_correction_ = work.output.map_camera_init;
      latest_manager_correction_stamp_ns_ = work.output.correction_stamp_ns;
    }
    try {
      MatchEvidence evidence{
        eventStamp(), true, work.output.match_accepted, work.output.valid,
        correction_age_ns, EvidenceSource::kNormal, work.output.candidate_id,
        work.output.registration};
      evidence.normal_match_kind = work.output.matching_mode == MapMatchingMode::kFineOnly ?
        NormalMatchKind::kFineOnly : NormalMatchKind::kFullRecovery;
      const auto monitor_status = monitor_->update(evidence);
      publishMonitorStatus(monitor_status);
      tf_allowed_.store(monitor_status.state == LocalizationState::kLocalized ||
        monitor_status.state == LocalizationState::kDegraded);
      if (monitor_status.state == LocalizationState::kLost) {
        beginGlobalRelocalization(work.output.stamp_ns);
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Rejected normal matching result: %s", error.what());
    }
  }

  void publishLocalizedOutput(const LocalizationOutput & output)
  {
    const auto stamp = stampMessage(output.stamp_ns);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose = poseMessage(output.map_body);
    pose_publisher_->publish(pose);
    nav_msgs::msg::Odometry localized;
    localized.header = pose.header;
    localized.child_frame_id = tracking_frame_;
    localized.pose.pose = pose.pose;
    odometry_publisher_->publish(localized);
    path_.header = pose.header;
    path_.poses.push_back(pose);
    if (path_.poses.size() > max_path_poses_) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() +
        static_cast<std::ptrdiff_t>(path_.poses.size() - max_path_poses_));
    }
    path_publisher_->publish(path_);
  }

  void inputCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    const auto received_steady_ns = steadyNanoseconds();
    const auto input = adapt(*odometry, *cloud, odom_frame_, tracking_frame_);
    if (!input.success) {
      publishStatus(nanoseconds(odometry->header.stamp), false, input.reason, 0U, {});
      return;
    }
    const auto current_state = monitor_->latest().state;
    if (input.frame.stamp_ns <= latest_input_stamp_ns_.load()) {
      publishStatus(input.frame.stamp_ns, false, "non_monotonic_frontend_frame", 0U, {});
      return;
    }
    latest_input_stamp_ns_.store(input.frame.stamp_ns);
    last_input_steady_ns_ = steadyNanoseconds();
    latest_input_steady_ns_ = received_steady_ns;
    latest_preprocess_ms_ = static_cast<double>(steadyNanoseconds() - received_steady_ns) / 1e6;
    if (current_state == LocalizationState::kInitializing &&
      global_relocalization_enabled_)
    {
      if (beginInitialGlobalLocalization(input.frame.stamp_ns)) {
        // The session start stamp must precede every observed worker result.
        return;
      }
      if (!relocalization_worker_->enqueue(input.frame)) {
        publishStatus(
          input.frame.stamp_ns, false, "relocalization_queue_full", 0U, {},
          "global_relocalization");
      }
      return;
    }
    if (current_state == LocalizationState::kLost && global_relocalization_enabled_) {
      beginGlobalRelocalization(input.frame.stamp_ns);
      return;
    }
    if (current_state == LocalizationState::kRelocalizing) {
      if (!relocalization_worker_->enqueue(input.frame)) {
        publishStatus(
          input.frame.stamp_ns, false, "relocalization_queue_full", 0U, {},
          "global_relocalization");
      }
      return;
    }
    try {
      latest_match_frame_ = input.frame;
      const auto output = manager_->propagate(input.frame);
      if (output.valid) {
        std::lock_guard<std::mutex> lock{correction_mutex_};
        latest_correction_ = output.map_camera_init;
        latest_manager_correction_stamp_ns_ = output.correction_stamp_ns;
      }
      if (!output.valid || !tf_allowed_.load()) {
        return;
      }
      publishLocalizedOutput(output);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Rejected Localization input: %s", error.what());
    }
  }

  void publishMonitorStatus(const LocalizationStatusSnapshot & status)
  {
    a2w_fastlio_msgs::msg::LocalizationStatus message;
    message.stamp = stampMessage(status.stamp_ns);
    message.state = static_cast<std::uint8_t>(status.state);
    message.state_label = localizationStateName(status.state);
    message.reason = status.reason;
    message.consecutive_successes = static_cast<std::uint32_t>(status.consecutive_successes);
    message.consecutive_failures = static_cast<std::uint32_t>(status.consecutive_failures);
    message.relocalization_successes = static_cast<std::uint32_t>(
      status.relocalization_successes);
    message.correction_available = status.correction_available;
    message.correction_age_ns = status.correction_age_ns;
    message.candidate_id = status.candidate_id;
    message.fitness = status.registration.fitness;
    message.overlap = status.registration.overlap;
    message.correspondence_count = status.registration.correspondence_count;
    message.hardware_validation_pending = status.hardware_validation_pending;
    monitor_status_publisher_->publish(message);
  }

  void publishStatus(
    const std::int64_t stamp, const bool accepted, const std::string & reason,
    const std::uint64_t candidate_id,
    const a2w_fastlio_common::RegistrationResult & registration,
    const std::string & stage = "localization")
  {
    a2w_fastlio_msgs::msg::RegistrationStatus message;
    message.stamp = stampMessage(stamp);
    message.keyframe_id = 0U;
    message.candidate_id = candidate_id;
    message.accepted = accepted;
    message.stage = stage;
    message.reason = reason;
    message.fitness = registration.fitness;
    message.overlap = registration.overlap;
    message.correspondence_count = registration.correspondence_count;
    message.elapsed_ms = registration.elapsed_ms;
    status_publisher_->publish(message);
  }

  void observeOwner(const a2w_fastlio_msgs::msg::GlobalTfOwner & message)
  {
    if (owner_state_->observe({
        nanoseconds(message.stamp), message.owner_id, message.mode,
        message.parent_frame, message.child_frame, message.active}, now().nanoseconds()))
    {
      RCLCPP_ERROR(get_logger(), "Global TF disabled: a foreign owner is active");
    }
  }

  void heartbeat()
  {
    const auto current = now();
    a2w_fastlio_msgs::msg::GlobalTfOwner message;
    message.stamp = static_cast<builtin_interfaces::msg::Time>(current);
    message.owner_id = owner_id_;
    message.mode = "localization";
    message.parent_frame = map_frame_;
    message.child_frame = odom_frame_;
    message.active = true;
    owner_publisher_->publish(message);
  }

  void publishGlobalTransform()
  {
    // Republish only the last accepted correction. This timer performs no
    // registration and preserves the existing validity / single-owner gate.
    const auto current = now();
    if (!tf_allowed_.load() || !owner_state_->mayPublish(current.nanoseconds())) {
      return;
    }
    std::optional<a2w_fastlio_common::Pose3d> correction;
    {
      std::lock_guard<std::mutex> lock{correction_mutex_};
      correction = latest_correction_;
    }
    if (!correction) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = static_cast<builtin_interfaces::msg::Time>(current);
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = correction->translation.x();
    transform.transform.translation.y = correction->translation.y();
    transform.transform.translation.z = correction->translation.z();
    transform.transform.rotation.w = correction->rotation.w();
    transform.transform.rotation.x = correction->rotation.x();
    transform.transform.rotation.y = correction->rotation.y();
    transform.transform.rotation.z = correction->rotation.z();
    tf_broadcaster_->sendTransform(transform);
  }

  std::string map_frame_, odom_frame_, tracking_frame_, owner_id_, owner_topic_;
  a2w_fastlio_map::MapBundle bundle_;
  std::shared_ptr<LocalMapSelector> selector_;
  std::shared_ptr<MapMatcher> matcher_;
  std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition_;
  a2w_fastlio_common::DefaultAlgorithmSuiteConfig algorithm_config_;
  std::shared_ptr<a2w_fastlio_map::KeyFrameDatabase> relocalization_keyframes_;
  std::shared_ptr<a2w_fastlio_map::DescriptorDatabase> relocalization_descriptors_;
  std::shared_ptr<GlobalRelocalizer> relocalizer_;
  std::unique_ptr<LocalizationManager> manager_;
  std::unique_ptr<LocalizationMonitor> monitor_;
  std::unique_ptr<NormalMatchMotionGate> normal_match_gate_;
  std::unique_ptr<NormalMatchingWorker> normal_matching_worker_;
  std::unique_ptr<RelocalizationSession> relocalization_session_;
  std::unique_ptr<RelocalizationWorker> relocalization_worker_;
  std::unique_ptr<a2w_fastlio_common::GlobalTfOwnerState> owner_state_;
  RelocalizationConfig relocalization_config_;
  RelocalizationConfig initialization_relocalization_config_;
  double relocalization_time_budget_ms_{5000.0};
  bool global_relocalization_enabled_{true};
  bool handoff_ab_enabled_{false};
  std::atomic_bool handoff_ab_pending_{false};
  std::atomic<std::int64_t> handoff_after_stamp_ns_{0};
  FineSelectionDiagnostic pending_fine_selection_;
  FineMatchDiagnostics fine_match_diagnostics_;
  std::unique_ptr<FineFailureCapture> fine_failure_capture_;
  mutable std::mutex correction_mutex_;
  std::optional<a2w_fastlio_common::Pose3d> latest_correction_;
  std::optional<std::int64_t> latest_manager_correction_stamp_ns_;
  std::optional<a2w_fastlio_common::FrontendFrame> latest_match_frame_;
  std::atomic<std::int64_t> latest_input_stamp_ns_{-1};
  std::int64_t latest_input_steady_ns_{0}, submitted_input_steady_ns_{0};
  std::int64_t submitted_dispatch_steady_ns_{0};
  double latest_preprocess_ms_{0.0}, submitted_preprocess_ms_{0.0};
  std::optional<std::int64_t> last_input_steady_ns_;
  std::int64_t input_timeout_ns_{1'000'000'000LL};
  std::mutex result_mutex_;
  std::optional<NormalMatchingWorkResult> pending_normal_result_;
  std::deque<RelocalizationWorkResult> pending_relocalization_results_;
  std::atomic_bool tf_allowed_{false};
  std::atomic_bool initialization_relocalization_active_{false};
  std::size_t max_path_poses_{10000U};
  nav_msgs::msg::Path path_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  std::shared_ptr<Synchronizer> synchronizer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::RegistrationStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::LocalizationStatus>::SharedPtr
    monitor_status_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::GlobalTfOwner>::SharedPtr owner_publisher_;
  rclcpp::Subscription<a2w_fastlio_msgs::msg::GlobalTfOwner>::SharedPtr owner_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr owner_timer_;
  rclcpp::TimerBase::SharedPtr global_tf_timer_;
  rclcpp::TimerBase::SharedPtr normal_match_timer_;
  rclcpp::TimerBase::SharedPtr result_timer_;
};

}  // namespace a2w_fastlio_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<a2w_fastlio_localization::LocalizationNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

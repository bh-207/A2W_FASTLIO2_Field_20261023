#define A2W_TRACE_ROS 1
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "startup_trace.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <a2w_fastlio_msgs/msg/global_tf_owner.hpp>
#include <a2w_fastlio_msgs/msg/registration_status.hpp>
#include <a2w_fastlio_msgs/srv/save_map_bundle.hpp>
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
#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_map/map_io.hpp"
#include "a2w_fastlio_mapping/committed_loop_audit.hpp"
#include "a2w_fastlio_mapping/frontend_message_adapter.hpp"
#include "a2w_fastlio_mapping/global_tf_ownership.hpp"
#include "a2w_fastlio_mapping/map_odom_manager.hpp"
#include "a2w_fastlio_mapping/loop_pipeline.hpp"
#include "a2w_fastlio_mapping/map_bundle_service.hpp"
#include "a2w_fastlio_mapping/optimized_map_builder.hpp"
#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

// Signal handlers only set a flag; ROS callbacks perform the actual drain.
volatile std::sig_atomic_t shutdown_signal = 0;

void requestShutdown(int signal)
{
  shutdown_signal = signal;
}

std::int64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

builtin_interfaces::msg::Time toTimeMessage(const rclcpp::Time & time)
{
  return static_cast<builtin_interfaces::msg::Time>(time);
}

geometry_msgs::msg::Pose toPoseMessage(const a2w_fastlio_common::Pose3d & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.translation.x();
  message.position.y = pose.translation.y();
  message.position.z = pose.translation.z();
  message.orientation.w = pose.rotation.w();
  message.orientation.x = pose.rotation.x();
  message.orientation.y = pose.rotation.y();
  message.orientation.z = pose.rotation.z();
  return message;
}

std::string generatedUuid()
{
  static std::atomic<std::uint64_t> sequence{0U};
  const auto now = static_cast<std::uint64_t>(
    std::chrono::system_clock::now().time_since_epoch().count());
  const auto tail = now ^ (++sequence * 0x9e3779b97f4a7c15ULL);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0')
         << std::setw(8) << static_cast<std::uint32_t>(now >> 32U) << '-'
         << std::setw(4) << static_cast<std::uint16_t>(now >> 16U) << '-'
         << std::setw(4) << static_cast<std::uint16_t>((now & 0x0fffU) | 0x4000U) << '-'
         << std::setw(4) << static_cast<std::uint16_t>((tail >> 48U & 0x3fffU) | 0x8000U) << '-'
         << std::setw(12) << (tail & 0xffffffffffffULL);
  return stream.str();
}

std::string utcNow()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&time, &utc);
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

class VectorKeyFrameProvider final : public a2w_fastlio_common::KeyFrameProvider
{
public:
  void append(a2w_fastlio_common::KeyFrame keyframe)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (keyframe.id != keyframes_.size()) {
      throw std::invalid_argument{"keyframes must be appended contiguously"};
    }
    keyframes_.push_back(std::move(keyframe));
  }

  std::optional<a2w_fastlio_common::KeyFrame> get(const std::uint64_t id) const override
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (id >= keyframes_.size()) {
      return std::nullopt;
    }
    return keyframes_[id];
  }

  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (keyframes_.empty()) {
      return std::nullopt;
    }
    return std::pair<std::uint64_t, std::uint64_t>{0U, keyframes_.size() - 1U};
  }

private:
  mutable std::mutex mutex_;
  std::vector<a2w_fastlio_common::KeyFrame> keyframes_;
};

}  // namespace

class MappingBackendNode final : public rclcpp::Node
{
public:
  MappingBackendNode()
  : Node("mapping_backend_node")
  {
    graph_ = std::make_shared<PoseGraphOptimizer>(poseGraphConfig());
    const auto ledger_path = declare_parameter<std::string>("factor_ledger_path", "");
    if (!ledger_path.empty()) {
      if (std::filesystem::exists(ledger_path)) {
        throw std::invalid_argument{"factor ledger must be a fresh path"};
      }
      factor_ledger_ = std::make_unique<std::ofstream>(ledger_path);
      if (!*factor_ledger_) {throw std::runtime_error{"cannot open exact factor ledger"};}
      loop_provenance_ = std::make_unique<std::ofstream>(ledger_path+".registration.jsonl");
      if (!*loop_provenance_) {throw std::runtime_error{"cannot open loop provenance"};}
      writeFactorLedgerCsvHeader(*factor_ledger_);
      graph_->setFactorLedgerObserver([this](const FactorLedgerRecord & record) {
        writeFactorLedgerCsv(*factor_ledger_, record);
        if (!*factor_ledger_) {throw std::runtime_error{"factor ledger write failed"};}
      });
    }
    map_builder_ = std::make_unique<OptimizedMapBuilder>(optimizedMapConfig());
    owner_state_ = std::make_unique<GlobalTfOwnerState>(ownerConfig());
    const auto algorithms =
      a2w_fastlio_common::createDefaultAlgorithmSuite(algorithmSuiteConfig());
    place_recognition_ = algorithms.place_recognition;
    const auto local_map_config = localMapConfig();
    loop_pipeline_ = std::make_unique<LoopPipeline>(
      algorithms.place_recognition, algorithms.descriptor_index, algorithms.registration,
      graph_, a2w_fastlio_common::LocalMapBuilder{local_map_config}, loopPipelineConfig());

    const auto keyframe_odom_topic =
      declare_parameter<std::string>("keyframe_odom_topic", "/mapping/keyframe_odom");
    const auto keyframe_cloud_topic =
      declare_parameter<std::string>("keyframe_cloud_topic", "/mapping/keyframe_cloud");
    const auto optimized_odom_topic =
      declare_parameter<std::string>("optimized_odom_topic", "/mapping/optimized_odom");
    const auto optimized_path_topic =
      declare_parameter<std::string>("optimized_path_topic", "/mapping/optimized_path");
    const auto preview_topic = declare_parameter<std::string>(
      "optimized_map_preview_topic", "/mapping/optimized_map_preview");
    const auto registration_status_topic = declare_parameter<std::string>(
      "registration_status_topic", "/mapping/registration_status");
    owner_topic_ = declare_parameter<std::string>(
      "global_tf_owner_topic", "/a2w_fastlio/global_tf_owner");
    if (keyframe_odom_topic.empty() || keyframe_cloud_topic.empty() ||
      optimized_odom_topic.empty() || optimized_path_topic.empty() || preview_topic.empty() ||
      registration_status_topic.empty() || owner_topic_.empty())
    {
      throw std::invalid_argument{"mapping Topic parameters must not be empty"};
    }

    const auto input_qos = qos("input", 20, false);
    const auto optimized_qos = qos("optimized", 20, false);
    const auto latched_qos = qos("preview", 1, true);
    const auto owner_qos = qos("owner", 10, true);

    optimized_odom_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(optimized_odom_topic, optimized_qos);
    optimized_path_publisher_ =
      create_publisher<nav_msgs::msg::Path>(optimized_path_topic, latched_qos);
    preview_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(preview_topic, latched_qos);
    registration_status_publisher_ =
      create_publisher<a2w_fastlio_msgs::msg::RegistrationStatus>(
      registration_status_topic, optimized_qos);
    owner_publisher_ = create_publisher<a2w_fastlio_msgs::msg::GlobalTfOwner>(
      owner_topic_, owner_qos);
    owner_subscription_ = create_subscription<a2w_fastlio_msgs::msg::GlobalTfOwner>(
      owner_topic_, owner_qos,
      std::bind(&MappingBackendNode::observeOwner, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto sync_queue_size = declare_parameter<int>("sync_queue_size", 20);
    if (sync_queue_size <= 0) {
      throw std::invalid_argument{"sync_queue_size must be positive"};
    }
    odometry_subscriber_.subscribe(this, keyframe_odom_topic, input_qos.get_rmw_qos_profile());
    cloud_subscriber_.subscribe(this, keyframe_cloud_topic, input_qos.get_rmw_qos_profile());
    odometry_subscriber_.registerCallback([](const nav_msgs::msg::Odometry::ConstSharedPtr& m){offline_handoff::event("backend_odom_received",offline_handoff::stamp_ns(m->header.stamp));});
    cloud_subscriber_.registerCallback([](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){offline_handoff::event("backend_cloud_received",offline_handoff::stamp_ns(m->header.stamp));});
    synchronizer_ = std::make_shared<Synchronizer>(
      odometry_subscriber_, cloud_subscriber_, static_cast<std::uint32_t>(sync_queue_size));
    synchronizer_->registerCallback(
      std::bind(&MappingBackendNode::keyframeCallback, this,
      std::placeholders::_1, std::placeholders::_2));

    const auto heartbeat_ms = declare_parameter<int>("owner_heartbeat_ms", 250);
    if (heartbeat_ms <= 0) {
      throw std::invalid_argument{"owner_heartbeat_ms must be positive"};
    }
    owner_state_->start(now().nanoseconds());
    owner_timer_ = create_wall_timer(
      std::chrono::milliseconds{heartbeat_ms}, std::bind(&MappingBackendNode::heartbeat, this));
    const auto worker_period_ms = declare_parameter<int>("worker_period_ms", 10);
    if (worker_period_ms <= 0) {
      throw std::invalid_argument{"worker_period_ms must be positive"};
    }
    worker_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    worker_timer_ = create_wall_timer(
      std::chrono::milliseconds{worker_period_ms},
      std::bind(&MappingBackendNode::processOneKeyframe, this), worker_callback_group_);

    const auto preview_ms = declare_parameter<int>("preview_period_ms", 5000);
    const auto status_ms = declare_parameter<int>("status_period_ms", 5000);
    if (preview_ms <= 0 || status_ms <= 0) {
      throw std::invalid_argument{"preview/status periods must be positive"};
    }
    preview_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    preview_timer_ = create_wall_timer(
      std::chrono::milliseconds{preview_ms},
      std::bind(&MappingBackendNode::publishPreview, this), preview_callback_group_);
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds{status_ms}, std::bind(&MappingBackendNode::reportProgress, this));
    shutdown_timer_ = create_wall_timer(
      std::chrono::milliseconds{100}, std::bind(&MappingBackendNode::checkShutdown, this));

    const auto bundle_root = declare_parameter<std::string>("bundle_root", "maps");
    const auto save_queue_capacity = declare_parameter<int>("save_queue_capacity", 1);
    if (bundle_root.empty() || save_queue_capacity <= 0) {
      throw std::invalid_argument{"invalid Map Bundle save configuration"};
    }
    frontend_revision_ = declare_parameter<std::string>(
      "frontend_revision", "16e97cdcc4260b9ba6241518a19ad8384224ba48");
    scan_context_revision_ = declare_parameter<std::string>("scan_context_revision", "curated");
    quatro_revision_ = declare_parameter<std::string>("quatro_revision", "d27109b");
    nano_gicp_revision_ = declare_parameter<std::string>("nano_gicp_revision", "b21e79e");
    teaser_revision_ = declare_parameter<std::string>("teaser_revision", "974574c");
    pmc_revision_ = declare_parameter<std::string>("pmc_revision", "a2dfd61");
    bundle_service_ = std::make_unique<MapBundleService>(
      bundle_root, static_cast<std::size_t>(save_queue_capacity));
    bundle_root_ = bundle_root;
    save_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    save_service_ = create_service<a2w_fastlio_msgs::srv::SaveMapBundle>(
      declare_parameter<std::string>("save_map_bundle_service", "/mapping/save_map_bundle"),
      std::bind(&MappingBackendNode::saveMapBundleResponse, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      rmw_qos_profile_services_default, save_callback_group_);
  }

  int finalizeExit()
  {
    // Called only after executor.spin() has joined its workers. A signalled
    // process that stopped with failed/incomplete work must not report exit0.
    std::lock_guard<std::mutex> lock{progress_mutex_};
    bool complete = failure_.empty() && active_save_callbacks_ == 0U &&
      !save_in_progress_ && !active_keyframe_ && processed_keyframes_ == keyframe_count_.load();
    for (auto * stream : {factor_ledger_.get(), loop_provenance_.get()}) {
      if (stream) {
        stream->flush();
        stream->close();
        complete = complete && static_cast<bool>(*stream);
      }
    }
    return complete ? 0 : 2;
  }

private:
  using Synchronizer = message_filters::TimeSynchronizer<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  PoseGraphConfig poseGraphConfig()
  {
    PoseGraphConfig config;
    config.prior_rotation_sigma_rad =
      declare_parameter<double>("pose_graph.prior_rotation_sigma_rad", config.prior_rotation_sigma_rad);
    config.prior_translation_sigma_m =
      declare_parameter<double>("pose_graph.prior_translation_sigma_m", config.prior_translation_sigma_m);
    config.odometry_rotation_sigma_rad =
      declare_parameter<double>("pose_graph.odometry_rotation_sigma_rad", config.odometry_rotation_sigma_rad);
    config.odometry_translation_sigma_m = declare_parameter<double>(
      "pose_graph.odometry_translation_sigma_m", config.odometry_translation_sigma_m);
    config.loop_rotation_sigma_rad =
      declare_parameter<double>("pose_graph.loop_rotation_sigma_rad", config.loop_rotation_sigma_rad);
    config.loop_translation_sigma_m =
      declare_parameter<double>("pose_graph.loop_translation_sigma_m", config.loop_translation_sigma_m);
    const auto robust_kernel =
      declare_parameter<std::string>("pose_graph.robust_kernel", "huber");
    if (robust_kernel == "huber") {
      config.robust_kernel = RobustKernel::HUBER;
    } else if (robust_kernel == "cauchy") {
      config.robust_kernel = RobustKernel::CAUCHY;
    } else {
      throw std::invalid_argument{"pose_graph.robust_kernel must be huber or cauchy"};
    }
    config.robust_kernel_scale =
      declare_parameter<double>("pose_graph.robust_kernel_scale", config.robust_kernel_scale);
    config.relinearization_threshold = declare_parameter<double>(
      "pose_graph.relinearization_threshold", config.relinearization_threshold);
    config.relinearization_skip =
      declare_parameter<int>("pose_graph.relinearization_skip", config.relinearization_skip);
    return config;
  }

  OptimizedMapConfig optimizedMapConfig()
  {
    OptimizedMapConfig config;
    config.full_voxel_leaf_m =
      declare_parameter<double>("full_map_voxel_leaf_m", config.full_voxel_leaf_m);
    config.preview_voxel_leaf_m =
      declare_parameter<double>("preview_voxel_leaf_m", config.preview_voxel_leaf_m);
    const auto max_points =
      declare_parameter<int>("preview_max_points", static_cast<int>(config.preview_max_points));
    if (max_points <= 0) {
      throw std::invalid_argument{"preview_max_points must be positive"};
    }
    config.preview_max_points = static_cast<std::size_t>(max_points);
    return config;
  }

  a2w_fastlio_common::DefaultAlgorithmSuiteConfig algorithmSuiteConfig()
  {
    a2w_fastlio_common::DefaultAlgorithmSuiteConfig config;
    const auto rings = declare_parameter<int>(
      "scan_context.rings", static_cast<int>(config.place_recognition.rings));
    const auto sectors = declare_parameter<int>(
      "scan_context.sectors", static_cast<int>(config.place_recognition.sectors));
    if (rings <= 0 || sectors <= 0) {
      throw std::invalid_argument{"Scan Context dimensions must be positive"};
    }
    config.place_recognition.rings = static_cast<std::size_t>(rings);
    config.place_recognition.sectors = static_cast<std::size_t>(sectors);
    config.place_recognition.max_radius_m = declare_parameter<double>(
      "scan_context.max_radius_m", config.place_recognition.max_radius_m);
    config.place_recognition.sensor_height_m = declare_parameter<double>(
      "scan_context.sensor_height_m", config.place_recognition.sensor_height_m);

    config.coarse.normal_radius_m =
      declare_parameter<double>("quatro.fpfh_normal_radius_m", config.coarse.normal_radius_m);
    config.coarse.feature_radius_m =
      declare_parameter<double>("quatro.fpfh_radius_m", config.coarse.feature_radius_m);
    config.coarse.noise_bound_m =
      declare_parameter<double>("quatro.noise_bound_m", config.coarse.noise_bound_m);
    config.coarse.rotation_gnc_factor = declare_parameter<double>(
      "quatro.rotation_gnc_factor", config.coarse.rotation_gnc_factor);
    config.coarse.rotation_cost_threshold = declare_parameter<double>(
      "quatro.rotation_cost_threshold", config.coarse.rotation_cost_threshold);
    config.coarse.rotation_max_iterations = declare_parameter<int>(
      "quatro.rotation_max_iterations", config.coarse.rotation_max_iterations);
    config.coarse.estimate_scale =
      declare_parameter<bool>("quatro.estimate_scale", config.coarse.estimate_scale);
    config.coarse.optimized_matching = declare_parameter<bool>(
      "quatro.optimized_matching", config.coarse.optimized_matching);
    config.coarse.descriptor_distance_threshold = declare_parameter<double>(
      "quatro.descriptor_distance_threshold", config.coarse.descriptor_distance_threshold);
    config.coarse.maximum_correspondences = declare_parameter<int>(
      "quatro.maximum_correspondences", config.coarse.maximum_correspondences);
    const auto coarse_minimum_points = declare_parameter<int>(
      "quatro.minimum_points", static_cast<int>(config.coarse.minimum_points));
    if (coarse_minimum_points <= 0) {
      throw std::invalid_argument{"quatro.minimum_points must be positive"};
    }
    config.coarse.minimum_points = static_cast<std::size_t>(coarse_minimum_points);

    config.fine.maximum_correspondence_distance_m = declare_parameter<double>(
      "nano_gicp.maximum_correspondence_distance_m",
      config.fine.maximum_correspondence_distance_m);
    config.fine.thread_count =
      declare_parameter<int>("nano_gicp.thread_count", config.fine.thread_count);
    config.fine.correspondence_randomness = declare_parameter<int>(
      "nano_gicp.correspondence_randomness", config.fine.correspondence_randomness);
    config.fine.maximum_iterations = declare_parameter<int>(
      "nano_gicp.maximum_iterations", config.fine.maximum_iterations);
    config.fine.transformation_epsilon = declare_parameter<double>(
      "nano_gicp.transformation_epsilon", config.fine.transformation_epsilon);
    config.fine.rotation_epsilon =
      declare_parameter<double>("nano_gicp.rotation_epsilon", config.fine.rotation_epsilon);
    config.fine.regularization_method = declare_parameter<int>(
      "nano_gicp.regularization_method", config.fine.regularization_method);
    config.fine.fitness_score_max_range_m = declare_parameter<double>(
      "nano_gicp.fitness_score_max_range_m", config.fine.fitness_score_max_range_m);
    const auto fine_minimum_points = declare_parameter<int>(
      "nano_gicp.minimum_points", static_cast<int>(config.fine.minimum_points));
    if (fine_minimum_points <= 0) {
      throw std::invalid_argument{"nano_gicp.minimum_points must be positive"};
    }
    config.fine.minimum_points = static_cast<std::size_t>(fine_minimum_points);

    config.validation.maximum_fitness = declare_parameter<double>(
      "loop_validation.maximum_fitness", config.validation.maximum_fitness);
    config.validation.minimum_overlap = declare_parameter<double>(
      "loop_validation.minimum_overlap", config.validation.minimum_overlap);
    const auto minimum_correspondences = declare_parameter<int>(
      "loop_validation.minimum_correspondences",
      static_cast<int>(config.validation.minimum_correspondences));
    if (minimum_correspondences <= 0) {
      throw std::invalid_argument{"loop_validation.minimum_correspondences must be positive"};
    }
    config.validation.minimum_correspondences =
      static_cast<std::size_t>(minimum_correspondences);
    config.validation.maximum_translation_jump_m = declare_parameter<double>(
      "loop_validation.maximum_translation_jump_m",
      config.validation.maximum_translation_jump_m);
    config.validation.maximum_rotation_jump_rad = declare_parameter<double>(
      "loop_validation.maximum_rotation_jump_rad",
      config.validation.maximum_rotation_jump_rad);
    config.validation.minimum_candidate_distance_separation = declare_parameter<double>(
      "loop_validation.minimum_candidate_distance_separation",
      config.validation.minimum_candidate_distance_separation);
    config.evidence_distance_m =
      declare_parameter<double>("pipeline.evidence_distance_m", config.evidence_distance_m);
    return config;
  }

  a2w_fastlio_common::LocalMapConfig localMapConfig()
  {
    a2w_fastlio_common::LocalMapConfig config;
    config.voxel_leaf_m =
      declare_parameter<double>("local_map.voxel_leaf_m", config.voxel_leaf_m);
    const auto max_points =
      declare_parameter<int>("local_map.max_points", static_cast<int>(config.max_points));
    if (max_points <= 0) {
      throw std::invalid_argument{"local_map.max_points must be positive"};
    }
    config.max_points = static_cast<std::size_t>(max_points);
    return config;
  }

  LoopPipelineConfig loopPipelineConfig()
  {
    LoopPipelineConfig config;
    config.causal_geometry_guard_enabled=declare_parameter<bool>("loop_pipeline.causal_geometry_guard_enabled",false);
    config.causal_geometry_guard_ledger_path=declare_parameter<std::string>("loop_pipeline.causal_geometry_guard_ledger_path","");
    const auto positive_size = [this](const std::string & name, const std::size_t value) {
        const auto parameter = declare_parameter<int>(name, static_cast<int>(value));
        if (parameter <= 0) {
          throw std::invalid_argument{name + " must be positive"};
        }
        return static_cast<std::size_t>(parameter);
      };
    const auto nonnegative_size = [this](const std::string & name, const std::size_t value) {
        const auto parameter = declare_parameter<int>(name, static_cast<int>(value));
        if (parameter < 0) {
          throw std::invalid_argument{name + " must not be negative"};
        }
        return static_cast<std::size_t>(parameter);
      };
    config.top_k = positive_size("scan_context.top_k", config.top_k);
    config.exclude_recent = nonnegative_size("scan_context.exclude_recent", config.exclude_recent);
    config.local_map_before = nonnegative_size(
      "local_map.neighbor_keyframes_before", config.local_map_before);
    config.local_map_after = nonnegative_size(
      "local_map.neighbor_keyframes_after", config.local_map_after);
    config.minimum_keyframes_between_accepted_loops = positive_size(
      "loop_pipeline.minimum_keyframes_between_accepted_loops",
      config.minimum_keyframes_between_accepted_loops);
    config.queue_capacity = positive_size("loop_pipeline.queue_capacity", config.queue_capacity);
    config.maximum_descriptor_distance = declare_parameter<double>(
      "scan_context.max_distance", config.maximum_descriptor_distance);
    config.revisit_verification_enabled = declare_parameter<bool>(
      "loop_pipeline.revisit_verification_enabled", config.revisit_verification_enabled);
    config.origin_keyframe_count = positive_size(
      "loop_pipeline.origin_keyframe_count", config.origin_keyframe_count);
    config.origin_top_k = positive_size("loop_pipeline.origin_top_k", config.origin_top_k);
    config.hypothesis_group_id_span = nonnegative_size(
      "loop_pipeline.hypothesis_group_id_span", config.hypothesis_group_id_span);
    config.hypothesis_group_distance_m = declare_parameter<double>(
      "loop_pipeline.hypothesis_group_distance_m", config.hypothesis_group_distance_m);
    config.solution_merge_distance_m = declare_parameter<double>(
      "loop_pipeline.solution_merge_distance_m", config.solution_merge_distance_m);
    config.solution_merge_rotation_rad = declare_parameter<double>(
      "loop_pipeline.solution_merge_rotation_rad", config.solution_merge_rotation_rad);
    config.distinct_solution_fitness_margin_m = declare_parameter<double>(
      "loop_pipeline.distinct_solution_fitness_margin_m", config.distinct_solution_fitness_margin_m);
    config.confirmation_frames = positive_size(
      "loop_pipeline.confirmation_frames", config.confirmation_frames);
    config.confirmation_max_gap = positive_size(
      "loop_pipeline.confirmation_max_gap", config.confirmation_max_gap);
    config.retry_interval_keyframes = positive_size(
      "loop_pipeline.retry_interval_keyframes", config.retry_interval_keyframes);
    config.retry_max_attempts = positive_size(
      "loop_pipeline.retry_max_attempts", config.retry_max_attempts);
    config.retry_reset_keyframes = positive_size(
      "loop_pipeline.retry_reset_keyframes", config.retry_reset_keyframes);
    return config;
  }

  GlobalTfOwnerConfig ownerConfig()
  {
    owner_id_ = declare_parameter<std::string>("owner_id", "mapping_backend");
    mode_ = declare_parameter<std::string>("mode", "mapping");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "body");
    const auto conflict_window_ms = declare_parameter<int>("owner_conflict_window_ms", 1000);
    if (tracking_frame_.empty() || conflict_window_ms <= 0) {
      throw std::invalid_argument{"invalid tracking frame or owner conflict window"};
    }
    return {
      owner_id_, mode_, map_frame_, odom_frame_,
      static_cast<std::int64_t>(conflict_window_ms) * 1'000'000LL};
  }

  rclcpp::QoS qos(const std::string & prefix, const int default_depth, const bool transient)
  {
    const auto depth = declare_parameter<int>(prefix + "_qos_depth", default_depth);
    const auto reliability =
      declare_parameter<std::string>(prefix + "_qos_reliability", "reliable");
    const auto durability = declare_parameter<std::string>(
      prefix + "_qos_durability", transient ? "transient_local" : "volatile");
    if (depth <= 0 || (reliability != "reliable" && reliability != "best_effort") ||
      (durability != "volatile" && durability != "transient_local"))
    {
      throw std::invalid_argument{"invalid QoS parameters for " + prefix};
    }
    auto result = rclcpp::QoS{rclcpp::KeepLast{static_cast<std::size_t>(depth)}};
    reliability == "reliable" ? result.reliable() : result.best_effort();
    durability == "transient_local" ? result.transient_local() : result.durability_volatile();
    return result;
  }

  void keyframeCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    offline_handoff::event("backend_synchronized",offline_handoff::stamp_ns(odometry->header.stamp));
    std::lock_guard<std::mutex> intake_lock{progress_mutex_};
    if (!accepting_) {
      offline_handoff::event("backend_ignored_after_freeze",offline_handoff::stamp_ns(odometry->header.stamp));
      ++ignored_after_freeze_;
      return;
    }
    const auto adapted = makeFrontendFrame(*odometry, *cloud, odom_frame_, tracking_frame_);
    if (!adapted.success) {
      // A corrupt input must not be silently excluded from a "complete" save.
      failure_ = "invalid_keyframe:" + adapted.error;
      accepting_ = false;
      preview_cancelled_ = true;
      progress_changed_.notify_all();
      RCLCPP_ERROR(get_logger(), "%s; complete save disabled", failure_.c_str());
      return;
    }
    if (last_stamp_ns_ && adapted.frame.stamp_ns <= *last_stamp_ns_) {
      return;
    }

    const auto id = static_cast<std::uint64_t>(keyframe_count_);
    a2w_fastlio_common::KeyFrame keyframe;
    keyframe.id = id;
    keyframe.stamp_ns = adapted.frame.stamp_ns;
    keyframe.odom_pose = adapted.frame.odom_pose;
    keyframe.optimized_pose = adapted.frame.odom_pose;
    keyframe.body_cloud = a2w_fastlio_common::CloudPtr{
      new a2w_fastlio_common::Cloud{*adapted.frame.body_cloud}};

    // The provider already retains every cloud for the final map. A processed cursor
    // makes it the backlog too, without a second bounded queue that can drop frames.
    provider_.append(std::move(keyframe));
    offline_handoff::event("backend_accepted",adapted.frame.stamp_ns,static_cast<std::int64_t>(id));
    ++keyframe_count_;
    last_stamp_ns_ = adapted.frame.stamp_ns;
  }

  void reportLoopProgress(
    LoopProgressStage stage, const LoopPipelineEvent & event, const std::int64_t sensor_stamp_ns,
    const std::chrono::steady_clock::time_point processing_started)
  {
    switch (stage) {
      case LoopProgressStage::CandidateDetected:
        RCLCPP_INFO(get_logger(),
          "[回环检测] 发现候选回环：当前关键帧 %lu -> 历史关键帧 %lu（排名 %zu）",
          event.current_id, event.candidate_id, event.candidate_rank);
        break;
      case LoopProgressStage::RegistrationStarted:
        RCLCPP_INFO(get_logger(),
          "[回环验证] %lu -> %lu：Scan Context 候选筛选通过，正在执行 Quatro / Nano-GICP / MatchValidator……",
          event.current_id, event.candidate_id);
        break;
      case LoopProgressStage::CandidateFinished:
        if (loop_provenance_ && event.evaluated_constraint) {
          auto & out=*loop_provenance_;out<<std::setprecision(17);
          const auto pose=[&out](const a2w_fastlio_common::Pose3d& p) {
            out<<'['<<p.translation.x()<<','<<p.translation.y()<<','<<p.translation.z()<<','
               <<p.rotation.x()<<','<<p.rotation.y()<<','<<p.rotation.z()<<','<<p.rotation.w()<<']';};
          const auto ids=[&out](const std::vector<std::uint64_t>& values) {
            out<<'[';bool first=true;for(const auto id:values){if(!first)out<<',';first=false;out<<id;}out<<']';};
          out<<"{\"stamp_ns\":"<<sensor_stamp_ns<<",\"from\":"<<event.current_id<<",\"to\":"<<event.candidate_id
             <<",\"measurement_direction\":\"T_body_current_body_history\",\"registration_frame\":\"map\",\"source_ids\":";
          ids(event.source_submap_ids);out<<",\"target_ids\":";ids(event.target_submap_ids);
          out<<",\"accepted\":"<<(event.accepted?"true":"false");
          out<<",\"measurement_tx_ty_tz_qxyzw\":";pose(event.evaluated_constraint->relative_pose);
          out<<",\"initial_guess\":";pose(event.registration_initial_guess);
          out<<",\"coarse\":";pose(event.coarse_transform);out<<",\"fine_D_map\":";pose(event.registration.transform);
          if(event.pose_check){out<<",\"predicted\":";pose(event.pose_check->predicted_pose);out<<",\"corrected\":";pose(event.pose_check->corrected_pose);}
          out<<"}\n";out.flush();
          if(!out)throw std::runtime_error{"loop provenance write failure"};
        }
        // Context-only rejections did not enter registration; do not print an
        // invented pose comparison for them. One diagnostic per evaluated pair.
        if (event.target_points > 0U) {
          RCLCPP_INFO(get_logger(), "%s",
            loopPoseCheckChinese(event, map_frame_, tracking_frame_).c_str());
        }
        if (event.accepted) {
          RCLCPP_INFO(get_logger(),
            "========== 回环验证成功 ==========\n"
            "[回环成功] 当前关键帧=%lu 匹配关键帧=%lu fitness=%.6f overlap=%.3f 对应点=%zu；约束待图优化提交。",
            event.current_id, event.candidate_id, event.registration.fitness,
            event.registration.overlap, event.registration.correspondence_count);
        } else {
          if (event.reason == "loop_confirmation_pending") {
            RCLCPP_INFO(get_logger(), "[回环确认] %lu -> %lu：几何验证通过，等待新关键帧一致性确认（已确认 %zu 帧），尚未加入图。",
              event.current_id, event.candidate_id, event.confirmation_count);
          } else if (event.target_points > 0U) {
            RCLCPP_WARN(get_logger(), "[回环失败] %lu -> %lu：%s",
              event.current_id, event.candidate_id, loopRejectionChinese(event.reason).c_str());
          }
        }
        break;
      case LoopProgressStage::OptimizationStarted:
        RCLCPP_INFO(get_logger(),
          "[图优化] 正在执行回环后的位姿图优化（%lu -> %lu），请保持建图程序运行……",
          event.current_id, event.candidate_id);
        break;
      case LoopProgressStage::OptimizationFinished:
        if (event.accepted && event.optimized_pose_change) {
          const auto commit_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
          const auto process_to_commit_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - processing_started).count();
          const auto audit = committedLoopAudit(
            stage, event, {sensor_stamp_ns, commit_wall_ns, process_to_commit_ms});
          RCLCPP_INFO(get_logger(), "%s", audit->c_str());
          ++committed_loop_count_;
          const auto & delta = *event.optimized_pose_change;
          RCLCPP_INFO(get_logger(),
            "[图优化完成] 回环约束已提交：%lu -> %lu；当前关键帧相对优化前预测位姿："
            "位置修正=%.3f m 旋转修正=%.3f deg；耗时=%.1f ms。"
            "这不是保存完成，请等待保存请求排空队列并写盘。",
            event.current_id, event.candidate_id, delta.translation.norm(),
            Eigen::AngleAxisd(delta.rotation).angle() * 180.0 / std::acos(-1.0), event.gtsam_ms);
        } else {
          RCLCPP_ERROR(get_logger(), "[图优化失败] 回环尚未确认提交：%s", event.reason.c_str());
        }
        break;
    }
  }

  void processOneKeyframe()
  {
    std::uint64_t id;
    {
      std::lock_guard<std::mutex> lock{progress_mutex_};
      if (!failure_.empty() || processed_keyframes_ == keyframe_count_.load()) {
        return;
      }
      id = processed_keyframes_;
      active_keyframe_ = id;
    }
    const auto started = std::chrono::steady_clock::now();
    try {
      std::lock_guard<std::mutex> backend_lock{backend_state_mutex_};
      const auto current = provider_.get(id);
      if (!current) {
        throw std::runtime_error{"pending keyframe is missing"};
      }
      const auto events = loop_pipeline_->process(*current,
        [this, sensor_stamp_ns = current->stamp_ns, started](auto stage, const auto & event) {
          reportLoopProgress(stage, event, sensor_stamp_ns, started);
        });
      for (const auto & event : events) {
        publishStatus(
          event.current_id, current->stamp_ns, event.accepted,
          "loop", event.reason, event.candidate_id, event.registration);
        RCLCPP_INFO(
          get_logger(),
          "[LoopProfile] current=%lu candidate=%lu rank=%zu candidates=%zu "
          "source_points=%zu target_points=%zu descriptor_ms=%.3f scan_context_ms=%.3f "
          "source_local_map_ms=%.3f target_local_map_ms=%.3f quatro_ms=%.3f "
          "nano_gicp_ms=%.3f evidence_ms=%.3f validator_ms=%.3f gtsam_ms=%.3f "
          "registration_total_ms=%.3f frame_total_ms=%.3f accepted=%s reason=%s "
          "sc_distance=%.6f yaw_hint_rad=%.4f origin_candidate=%s confirmed_frames=%zu stage=%s",
          event.current_id, event.candidate_id, event.candidate_rank, event.candidate_count,
          event.source_points, event.target_points, event.descriptor_ms,
          event.scan_context_ms, event.source_local_map_ms, event.target_local_map_ms,
          event.quatro_ms, event.nano_gicp_ms, event.evidence_ms, event.validator_ms,
          event.gtsam_ms, event.registration_total_ms, event.frame_total_ms,
          event.accepted ? "true" : "false", event.reason.c_str(),
          event.descriptor_distance, event.yaw_hint_rad,
          event.origin_candidate ? "true" : "false", event.confirmation_count,
          event.rejection_stage.c_str());
        if (event.accepted) {
          RCLCPP_INFO(get_logger(), "Loop accepted: current=%lu candidate=%lu factors=%zu",
            event.current_id, event.candidate_id, event.graph_factor_count);
        }
      }
      const auto snapshot = graph_->optimizedPoses();
      if (snapshot.poses.size() != id + 1U || snapshot.poses.back().id != id) {
        throw std::runtime_error{"pose graph did not commit current keyframe"};
      }
      correction_manager_.update(
        id, current->stamp_ns, snapshot.poses.back().pose, current->odom_pose);
      publishProducts(snapshot, current->stamp_ns);
      publishStatus(
        id, current->stamp_ns, true, "pose_graph", "optimized_snapshot_updated");
      {
        std::lock_guard<std::mutex> lock{progress_mutex_};
        offline_handoff::event("backend_processed",current->stamp_ns,static_cast<std::int64_t>(id));
        ++processed_keyframes_;
        active_keyframe_.reset();
        last_completed_stamp_ns_ = now().nanoseconds();
        last_completed_steady_ = std::chrono::steady_clock::now();
        last_processing_ms_ = std::chrono::duration<double, std::milli>(
          last_completed_steady_ - started).count();
      }
      progress_changed_.notify_all();
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock{progress_mutex_};
        failure_ = error.what();
        active_keyframe_.reset();
        accepting_ = false;
        preview_cancelled_ = true;
      }
      progress_changed_.notify_all();
      RCLCPP_ERROR(get_logger(), "Mapping processing failed at %lu: %s; save disabled",
        id, error.what());
    }
  }

  void reportProgress()
  {
    std::string summary;
    bool draining = false;
    {
      std::lock_guard<std::mutex> lock{progress_mutex_};
      const auto pending = keyframe_count_.load() - processed_keyframes_;
      const auto state = !failure_.empty() ? "fault" :
        (saved_ ? "saved" : (accepting_ ? "receiving" : "draining"));
      std::ostringstream stream;
      stream << "received=" << keyframe_count_.load()
             << " processed=" << processed_keyframes_ << " pending=" << pending
             << " active=" << (active_keyframe_ ? std::to_string(*active_keyframe_) : "none")
             << " state=" << state << " last_completed_stamp_ns=" << last_completed_stamp_ns_
             << " last_completed_age_ms=" << (last_completed_stamp_ns_ == 0 ? -1.0 :
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - last_completed_steady_).count())
             << " processing_ms=" << last_processing_ms_
             << " full_map_builds=" << full_map_builds_.load()
             << " preview_map_builds=" << preview_map_builds_.load()
             << " preview_ms=" << last_preview_ms_.load()
             << " ignored_after_freeze=" << ignored_after_freeze_;
      summary = stream.str();
      draining = save_in_progress_ && pending > 0U;
    }
    RCLCPP_INFO(get_logger(), "[MappingBackend] %s", summary.c_str());
    if (draining) {
      RCLCPP_INFO(get_logger(), "[保存等待] 正在排空建图队列，请勿退出：%s", summary.c_str());
    }
    publishStatus(0U, now().nanoseconds(), true, "backend", summary);
  }

  void checkShutdown()
  {
    if (shutdown_signal == 0) {
      return;
    }
    bool first = false;
    bool done = false;
    bool failed = false;
    {
      std::lock_guard<std::mutex> lock{progress_mutex_};
      first = !shutdown_requested_;
      shutdown_requested_ = true;
      accepting_ = false;
      preview_cancelled_ = true;
      failed = !failure_.empty();
      done = active_save_callbacks_ == 0U && !save_in_progress_ && !active_keyframe_ &&
        (failed || processed_keyframes_ == keyframe_count_.load());
    }
    if (first) {
      RCLCPP_WARN(get_logger(), "收到退出信号，停止接收并排空关键帧；退出本身不会自动保存地图。");
      reportProgress();
    }
    if (done) {
      reportProgress();
      RCLCPP_INFO(get_logger(), "%s", failed ? "shutdown_incomplete" : "shutdown_drained");
      rclcpp::shutdown();
    }
  }

  std::string parameterSnapshot(const std::vector<std::string> & prefixes)
  {
    auto names = list_parameters({}, 10U).names;
    std::sort(names.begin(), names.end());
    std::ostringstream stream;
    for (const auto & name : names) {
      const bool included = prefixes.empty() || std::any_of(
        prefixes.begin(), prefixes.end(), [&name](const auto & prefix) {
          return name.rfind(prefix, 0U) == 0U;
        });
      if (included) {
        stream << name << ": " << get_parameter(name).value_to_string() << '\n';
      }
    }
    return stream.str();
  }

  a2w_fastlio_map::MapBundleData bundleSnapshot()
  {
    a2w_fastlio_map::MapBundleData data;
    {
      std::lock_guard<std::mutex> backend_lock{backend_state_mutex_};
      {
        std::lock_guard<std::mutex> lock{progress_mutex_};
        if (accepting_ || active_keyframe_ || processed_keyframes_ != keyframe_count_.load() ||
          !failure_.empty())
        {
          throw std::runtime_error{"mapping_queue_not_drained"};
        }
      }
      // Flush a final iSAM2 update after all accepted keyframes/constraints have committed.
      RCLCPP_INFO(get_logger(), "[最终优化] 正在执行最终位姿图更新，请保持系统运行……");
      const auto final_update = graph_->update(true);
      if (!final_update.accepted) {
        throw std::runtime_error{"final_pose_graph_update_failed:" + final_update.reason};
      }
      const auto optimized = graph_->optimizedPoses();
      graph_->emitFactorLedger("final_audit");
      if (factor_ledger_) {
        factor_ledger_->flush();
        if (!*factor_ledger_) {throw std::runtime_error{"factor ledger flush failed"};}
      }
      if (loop_provenance_) {
        loop_provenance_->flush();
        if (!*loop_provenance_) {throw std::runtime_error{"loop provenance flush failed"};}
      }
      if (optimized.poses.empty() || optimized.poses.size() != keyframe_count_.load()) {
        throw std::runtime_error{"no optimized Mapping snapshot is available"};
      }
      RCLCPP_INFO(get_logger(), "[最终优化完成] 优化位姿已提交，关键帧=%zu 已提交回环=%zu；尚未写盘。",
        optimized.poses.size(), committed_loop_count_);
      if (committed_loop_count_ == 0U) {
        RCLCPP_WARN(get_logger(), "[回环状态] 尚无已提交的回环，本次保存不代表回环成功。");
      }
      data.keyframes.reserve(optimized.poses.size());
      for (const auto & pose : optimized.poses) {
        auto keyframe = provider_.get(pose.id);
        if (!keyframe) {
          throw std::runtime_error{"Mapping snapshot has a missing keyframe"};
        }
        keyframe->optimized_pose = pose.pose;
        data.keyframes.push_back(std::move(*keyframe));
      }
      ++full_map_builds_;
      RCLCPP_INFO(get_logger(), "[地图同步] 正在按最终优化位姿构建完整 global_map 并同步 Map Bundle……");
      data.global_map = map_builder_->buildFull(provider_, optimized);
      const auto & last = data.keyframes.back();
      correction_manager_.update(last.id, last.stamp_ns, last.optimized_pose, last.odom_pose, true);
      publishProducts(optimized, last.stamp_ns);
    }
    data.descriptors.reserve(data.keyframes.size());
    for (const auto & keyframe : data.keyframes) {
      data.descriptors.push_back({
        keyframe.id, place_recognition_->describe(keyframe.body_cloud)});
    }
    data.metadata.bundle_uuid = generatedUuid();
    data.metadata.created_utc = utcNow();
    data.metadata.map_frame = map_frame_;
    data.metadata.odom_frame = odom_frame_;
    data.metadata.tracking_frame = tracking_frame_;
    data.metadata.global_map_leaf_m = get_parameter("full_map_voxel_leaf_m").as_double();
    data.metadata.frontend_revision = frontend_revision_;
    data.metadata.dependency_revisions = {
      {"scan_context", scan_context_revision_}, {"quatro", quatro_revision_},
      {"nano_gicp", nano_gicp_revision_}, {"teaser++", teaser_revision_},
      {"pmc", pmc_revision_}, {"gtsam", "4.1.1"}};
    data.config_snapshots = {
      {"mapping_effective.yaml", parameterSnapshot({})},
      {"scan_context.yaml", parameterSnapshot({"scan_context."})},
      {"registration.yaml", parameterSnapshot(
          {"local_map.", "quatro.", "nano_gicp.", "pipeline.", "loop_validation.", "loop_pipeline."})},
    };
    return data;
  }

  void saveMapBundle(
    const std::shared_ptr<a2w_fastlio_msgs::srv::SaveMapBundle::Request> request,
    std::shared_ptr<a2w_fastlio_msgs::srv::SaveMapBundle::Response> response)
  {
    std::unique_lock<std::mutex> save_lock{save_mutex_, std::try_to_lock};
    if (!save_lock.owns_lock()) {
      response->message = "save_busy";
      return;
    }
    // Reject invalid paths before freezing this acquisition session.
    try {
      (void)a2w_fastlio_map::confinedPath(bundle_root_, request->output_path);
    } catch (const std::exception &) {
      response->message = "output_path_not_confined";
      return;
    }
    try {
      {
        std::unique_lock<std::mutex> lock{progress_mutex_};
        if (keyframe_count_ == 0U) {
          throw std::runtime_error{"no optimized Mapping snapshot is available"};
        }
        accepting_ = false;
        preview_cancelled_ = true;
        save_in_progress_ = true;
        RCLCPP_INFO(get_logger(), "[结束采集] 已冻结后端接收边界，正在处理剩余关键帧；不要关闭程序。");
        RCLCPP_INFO(get_logger(), "Draining mapping queue... received=%zu processed=%zu pending=%zu",
          keyframe_count_.load(), processed_keyframes_, keyframe_count_.load() - processed_keyframes_);
        while (failure_.empty() && rclcpp::ok() &&
          (active_keyframe_ || processed_keyframes_ != keyframe_count_.load()))
        {
          progress_changed_.wait_for(lock, std::chrono::milliseconds{100});
        }
        if (!failure_.empty()) {
          throw std::runtime_error{"mapping_processing_failed:" + failure_};
        }
        if (!rclcpp::ok()) {
          throw std::runtime_error{"mapping_shutdown_before_drain"};
        }
        RCLCPP_INFO(get_logger(),
          "[队列排空] received=%zu processed=%zu pending=0 active=none；回环随关键帧同步处理，无在途回环任务。",
          keyframe_count_.load(), processed_keyframes_);
      }
      // Wait for any cancelled preview to release its temporary cloud memory.
      std::lock_guard<std::mutex> preview_lock{preview_mutex_};
      auto data = bundleSnapshot();
      RCLCPP_INFO(get_logger(), "[地图就绪] 最终优化完成，位姿、点云及描述子已同步，正在写入 Map Bundle……");
      const auto result = bundle_service_->save(request->output_path, std::move(data));
      response->success = result.success;
      response->message = result.message;
      response->bundle_uuid = result.bundle_uuid;
      response->keyframe_count = result.keyframe_count;
      response->resolved_path = result.resolved_path.string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = error.what();
    }
    {
      std::lock_guard<std::mutex> lock{progress_mutex_};
      save_in_progress_ = false;
      saved_ = response->success;
    }
    reportProgress();
    RCLCPP_INFO(get_logger(), "Map Bundle 保存%s：%s", response->success ? "成功" : "失败",
      response->message.c_str());
    if (response->success) {
      RCLCPP_INFO(get_logger(), "[保存成功] 关键帧=%lu 路径=%s；可以关闭建图程序。",
        response->keyframe_count, response->resolved_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "[保存失败] %s；请保留程序运行并处理错误，不能视为建图完成。",
        response->message.c_str());
    }
  }

  void saveMapBundleResponse(
    std::shared_ptr<rclcpp::Service<a2w_fastlio_msgs::srv::SaveMapBundle>> service,
    std::shared_ptr<rmw_request_id_t> header,
    std::shared_ptr<a2w_fastlio_msgs::srv::SaveMapBundle::Request> request)
  {
    // Keep ROS alive until the response has actually been sent, even if SIGINT
    // arrived during draining or disk I/O. No extra service worker/queue is needed.
    ++active_save_callbacks_;
    auto response = std::make_shared<a2w_fastlio_msgs::srv::SaveMapBundle::Response>();
    try {
      saveMapBundle(request, response);
      service->send_response(*header, *response);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Save response failed: %s", error.what());
    }
    --active_save_callbacks_;
  }

  void publishProducts(const OptimizedPoseSnapshot & snapshot, const std::int64_t stamp_ns)
  {
    const auto stamp = toTimeMessage(rclcpp::Time{stamp_ns});
    const auto & current = snapshot.poses.back();
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = tracking_frame_;
    odometry.pose.pose = toPoseMessage(current.pose);
    optimized_odom_publisher_->publish(odometry);

    nav_msgs::msg::Path path;
    path.header = odometry.header;
    path.poses.reserve(snapshot.poses.size());
    for (const auto & optimized : snapshot.poses) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose = toPoseMessage(optimized.pose);
      path.poses.push_back(std::move(pose));
    }
    optimized_path_publisher_->publish(path);

    std::lock_guard<std::mutex> lock{snapshot_mutex_};
    preview_snapshot_ = snapshot;
    preview_stamp_ns_ = stamp_ns;
  }

  void publishPreview()
  {
    std::lock_guard<std::mutex> preview_lock{preview_mutex_};
    if (preview_cancelled_) {
      return;
    }
    OptimizedPoseSnapshot snapshot;
    std::int64_t stamp_ns;
    {
      std::lock_guard<std::mutex> lock{snapshot_mutex_};
      if (preview_snapshot_.poses.empty() ||
        preview_snapshot_.revision == last_preview_revision_)
      {
        return;
      }
      snapshot = preview_snapshot_;
      stamp_ns = preview_stamp_ns_;
    }
    const auto started = std::chrono::steady_clock::now();
    try {
      const auto preview = map_builder_->buildPreview(
        provider_, snapshot, [this]() {return preview_cancelled_.load();});
      if (preview_cancelled_) {
        return;
      }
      sensor_msgs::msg::PointCloud2 message;
      pcl::toROSMsg(*preview, message);
      message.header.stamp = toTimeMessage(rclcpp::Time{stamp_ns});
      message.header.frame_id = map_frame_;
      preview_publisher_->publish(message);
      last_preview_revision_ = snapshot.revision;
      ++preview_map_builds_;
      last_preview_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    } catch (const std::exception & error) {
      if (!preview_cancelled_) {
        RCLCPP_WARN(get_logger(), "Preview build failed: %s", error.what());
      }
    }
  }

  void publishStatus(
    const std::uint64_t id, const std::int64_t stamp_ns, const bool accepted,
    const std::string & stage, const std::string & reason,
    const std::uint64_t candidate_id = std::numeric_limits<std::uint64_t>::max(),
    const a2w_fastlio_common::RegistrationResult & registration = {})
  {
    a2w_fastlio_msgs::msg::RegistrationStatus status;
    status.stamp = toTimeMessage(rclcpp::Time{stamp_ns});
    status.keyframe_id = id;
    status.candidate_id = candidate_id;
    status.accepted = accepted;
    status.stage = stage;
    status.reason = reason;
    status.fitness = registration.fitness;
    status.overlap = registration.overlap;
    status.correspondence_count = registration.correspondence_count;
    status.elapsed_ms = registration.elapsed_ms;
    registration_status_publisher_->publish(status);
  }

  void observeOwner(const a2w_fastlio_msgs::msg::GlobalTfOwner & message)
  {
    const bool newly_conflicted = owner_state_->observe(
      {toNanoseconds(message.stamp), message.owner_id, message.mode,
        message.parent_frame, message.child_frame, message.active},
      now().nanoseconds());
    if (newly_conflicted && !fault_published_) {
      fault_published_ = true;
      publishStatus(
        keyframe_count_ == 0U ? 0U : keyframe_count_ - 1U, now().nanoseconds(), false,
        "ownership", owner_state_->faultReason(now().nanoseconds()));
      RCLCPP_ERROR(get_logger(), "Global TF disabled: a foreign owner is active");
    }
  }

  void heartbeat()
  {
    const auto current_time = now();
    a2w_fastlio_msgs::msg::GlobalTfOwner owner;
    owner.stamp = toTimeMessage(current_time);
    owner.owner_id = owner_id_;
    owner.mode = mode_;
    owner.parent_frame = map_frame_;
    owner.child_frame = odom_frame_;
    owner.active = true;
    owner_publisher_->publish(owner);

    const auto correction = correction_manager_.latest();
    if (!correction || !owner_state_->mayPublish(current_time.nanoseconds())) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = toTimeMessage(current_time);
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = correction->map_camera_init.translation.x();
    transform.transform.translation.y = correction->map_camera_init.translation.y();
    transform.transform.translation.z = correction->map_camera_init.translation.z();
    transform.transform.rotation.w = correction->map_camera_init.rotation.w();
    transform.transform.rotation.x = correction->map_camera_init.rotation.x();
    transform.transform.rotation.y = correction->map_camera_init.rotation.y();
    transform.transform.rotation.z = correction->map_camera_init.rotation.z();
    tf_broadcaster_->sendTransform(transform);
  }

  std::string owner_id_;
  std::string mode_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string tracking_frame_;
  std::string owner_topic_;
  std::unique_ptr<std::ofstream> factor_ledger_;
  std::unique_ptr<std::ofstream> loop_provenance_;
  std::shared_ptr<PoseGraphOptimizer> graph_;
  std::unique_ptr<OptimizedMapBuilder> map_builder_;
  std::unique_ptr<GlobalTfOwnerState> owner_state_;
  std::unique_ptr<LoopPipeline> loop_pipeline_;
  std::shared_ptr<a2w_fastlio_common::PlaceRecognition> place_recognition_;
  std::unique_ptr<MapBundleService> bundle_service_;
  MapOdomManager correction_manager_;
  VectorKeyFrameProvider provider_;
  std::atomic<std::size_t> keyframe_count_{0U};
  std::mutex progress_mutex_;
  std::condition_variable progress_changed_;
  std::size_t processed_keyframes_{0U};
  // Only accessed while holding backend_state_mutex_. Diagnostic count only.
  std::size_t committed_loop_count_{0U};
  std::optional<std::uint64_t> active_keyframe_;
  bool accepting_{true};
  bool shutdown_requested_{false};
  bool save_in_progress_{false};
  std::atomic<std::size_t> active_save_callbacks_{0U};
  bool saved_{false};
  std::size_t ignored_after_freeze_{0U};
  std::string failure_;
  std::int64_t last_completed_stamp_ns_{0};
  std::chrono::steady_clock::time_point last_completed_steady_{};
  double last_processing_ms_{0.0};
  std::atomic<std::size_t> full_map_builds_{0U};
  std::atomic<std::size_t> preview_map_builds_{0U};
  std::atomic<double> last_preview_ms_{0.0};
  std::atomic<bool> preview_cancelled_{false};
  std::mutex preview_mutex_;
  std::mutex snapshot_mutex_;
  OptimizedPoseSnapshot preview_snapshot_;
  std::uint64_t last_preview_revision_{0U};
  std::int64_t preview_stamp_ns_{0};
  std::mutex save_mutex_;
  std::filesystem::path bundle_root_;
  std::optional<std::int64_t> last_stamp_ns_;
  bool fault_published_{false};
  std::string frontend_revision_;
  std::string scan_context_revision_;
  std::string quatro_revision_;
  std::string nano_gicp_revision_;
  std::string teaser_revision_;
  std::string pmc_revision_;
  std::mutex backend_state_mutex_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  std::shared_ptr<Synchronizer> synchronizer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preview_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::RegistrationStatus>::SharedPtr
    registration_status_publisher_;
  rclcpp::Publisher<a2w_fastlio_msgs::msg::GlobalTfOwner>::SharedPtr owner_publisher_;
  rclcpp::Subscription<a2w_fastlio_msgs::msg::GlobalTfOwner>::SharedPtr owner_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr owner_timer_;
  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  rclcpp::CallbackGroup::SharedPtr preview_callback_group_;
  rclcpp::TimerBase::SharedPtr preview_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  rclcpp::CallbackGroup::SharedPtr save_callback_group_;
  rclcpp::Service<a2w_fastlio_msgs::srv::SaveMapBundle>::SharedPtr save_service_;
};

}  // namespace a2w_fastlio_mapping

int main(int argc, char ** argv)
{
    try { offline_handoff::start("backend"); } catch(const std::exception& e) { std::cerr<<e.what()<<"\n"; return 2; }

  rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, a2w_fastlio_mapping::requestShutdown);
  std::signal(SIGTERM, a2w_fastlio_mapping::requestShutdown);
  int exit_code = 2;
  try {
    auto node = std::make_shared<a2w_fastlio_mapping::MappingBackendNode>();
    // Input/heartbeat, registration, preview, and a waiting save must not starve each other.
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 4U};
    executor.add_node(node);
    executor.spin();
    exit_code = node->finalizeExit();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mapping_backend_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return exit_code;
}

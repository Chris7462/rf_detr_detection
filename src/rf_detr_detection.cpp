// C++ header
#include <string>
#include <chrono>
#include <functional>
#include <exception>

// ROS header
#include <ament_index_cpp/get_package_share_path.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

// local header
#include "rf_detr_detection/rf_detr_detection.hpp"
#include "rf_detr_trt_backend/detection_utils.hpp"


namespace rf_detr_detection
{

RfDetrDetection::RfDetrDetection()
: Node("rf_detr_detection_node"),
  processing_in_progress_(false)
{
  // Initialize ROS2 parameters with validation
  if (!initialize_parameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize parameters");
    rclcpp::shutdown();
    return;
  }

  // Initialize TensorRT inferencer
  if (!initialize_inferencer()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize TensorRT inferencer");
    rclcpp::shutdown();
    return;
  }

  // Initialize ROS2 components
  initialize_ros_components();

  RCLCPP_INFO(get_logger(),
    "RF-DETR Detection node initialized successfully with bounded queue (max: %d)",
    max_processing_queue_size_);
}

RfDetrDetection::~RfDetrDetection()
{
  RCLCPP_INFO(get_logger(), "RF-DETR Detection node shutting down");
}

bool RfDetrDetection::initialize_parameters()
{
  try {
    // ROS2 parameters
    input_topic_ = declare_parameter("input_topic",
      std::string("kitti/camera/color/left/image_raw"));
    output_topic_ = declare_parameter("output_topic", std::string("rf_detr_detection"));
    output_overlay_topic_ = declare_parameter("output_overlay_topic",
      std::string("rf_detr_detection_overlay"));
    queue_size_ = declare_parameter<int>("queue_size", 10);
    processing_frequency_ = declare_parameter<double>("processing_frequency", 40.0);

    // Processing queue parameter - small bounded queue for burst handling
    max_processing_queue_size_ = declare_parameter<int>("max_processing_queue_size", 3);

    // Declare and get parameters with validation
    std::string engine_package = declare_parameter("engine_package",
      std::string("rf_detr_trt_backend"));
    engine_filename_ = declare_parameter("engine_filename",
      std::string("rf_detr_large_704x704.engine"));

    // Model parameters. height/width must stay equal - RFDetrTrtBackend
    // enforces this (windowed backbone attention has no non-square path).
    config_.height = declare_parameter<int>("height", 704);
    config_.width = declare_parameter<int>("width", 704);
    config_.num_queries = declare_parameter<int>("num_queries", 300);
    config_.num_classes = declare_parameter<int>("num_classes", 90);
    config_.score_threshold = declare_parameter<double>("score_threshold", 0.5);
    config_.warmup_iterations = declare_parameter<int>("warmup_iterations", 2);
    // 0: Internal Error, 1: Error, 2: Warning, 3: Info, 4: Verbose
    int log_level = declare_parameter<int>("log_level", 2);
    config_.log_level = static_cast<rf_detr_trt_backend::LogLevel>(log_level);

    // Resolve engine path from the backend package's share/engines directory,
    // same pattern as fcn_segmentation.
    engine_path_ = ament_index_cpp::get_package_share_path(engine_package) /
      "engines" / engine_filename_;

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during parameter initialization: %s", e.what());
    return false;
  }
}

bool RfDetrDetection::initialize_inferencer()
{
  try {
    detector_ = std::make_shared<rf_detr_trt_backend::RFDetrTrtBackend>(
      engine_path_.string(), config_);
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception creating RFDetrTrtBackend: %s", e.what());
    return false;
  }
}

void RfDetrDetection::initialize_ros_components()
{
  // Configure QoS profile for reliable image transport
  rclcpp::QoS image_qos(queue_size_);
  image_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  image_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  image_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // Create a single REENTRANT callback group for all callbacks
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Create subscription options with dedicated callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  // Create subscriber
  img_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_, image_qos,
    std::bind(&RfDetrDetection::image_callback, this, std::placeholders::_1),
    sub_options);

  // Create publishers
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    output_topic_, image_qos);
  overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(output_overlay_topic_, image_qos);

  // Create timer for processing at specified frequency
  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&RfDetrDetection::timer_callback, this),
    callback_group_);

  RCLCPP_INFO(get_logger(), "ROS components initialized with separate callback groups");
  RCLCPP_INFO(get_logger(), "Input: %s, Output: %s, Frequency: %.1f Hz",
    input_topic_.c_str(), output_topic_.c_str(), processing_frequency_);
}

void RfDetrDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    // Thread-safe queue management
    std::lock_guard<std::mutex> lock(mtx_);

    // Check if queue is full
    if (img_buff_.size() >= static_cast<size_t>(max_processing_queue_size_)) {
      // Remove oldest image to make room for new one
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Processing queue full, dropping oldest image (queue size: %ld)", img_buff_.size());
      img_buff_.pop();
    }

    // Add new image to queue
    img_buff_.push(msg);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in image callback: %s", e.what());
  }
}

void RfDetrDetection::timer_callback()
{
  // Atomically claim the "processing" slot
  bool expected = false;
  if (!processing_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  // Get next image from queue
  sensor_msgs::msg::Image::SharedPtr msg;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (img_buff_.empty()) {
      processing_in_progress_.store(false);
      return;
    }
    msg = img_buff_.front();
    img_buff_.pop();
  }

  try {
    // Convert ROS image to OpenCV format. RFDetrTrtBackend's preprocessing
    // kernel hard-assumes BGR input (see preprocess_kernel.cu) - this
    // conversion is load-bearing, not a formality.
    cv_bridge::CvImageConstPtr cv_ptr =
      cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

    if (!cv_ptr || cv_ptr->image.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty or invalid image");
      processing_in_progress_.store(false);
      return;
    }

    // Process the image - letterbox/infer/unletterbox happens inside;
    // detections come back already in cv_ptr->image's own coordinates.
    rf_detr_trt_backend::Detections detections = process_image(cv_ptr->image);

    // Publish results
    if (detection_pub_->get_subscription_count() > 0) {
      publish_detection_result(detections, msg->header);
    }

    if (overlay_pub_->get_subscription_count() > 0) {
      cv::Mat overlay = rf_detr_trt_backend::utils::draw_detections(
        cv_ptr->image, detections);
      publish_overlay_result(overlay, msg->header);
    }

  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during image processing: %s", e.what());
  }

  // Clear processing flag
  processing_in_progress_.store(false);
}

rf_detr_trt_backend::Detections RfDetrDetection::process_image(const cv::Mat & input_image)
{
  if (input_image.empty()) {
    RCLCPP_WARN(get_logger(), "Input image is empty");
    return {};
  }

  try {
    // config_.height == config_.width (enforced by RFDetrTrtBackend's
    // constructor) - letterbox_square() pads/resizes to that square size.
    rf_detr_trt_backend::utils::LetterboxInfo info;
    cv::Mat padded = rf_detr_trt_backend::utils::letterbox_square(
      input_image, config_.width, &info);

    rf_detr_trt_backend::Detections detections_padded = detector_->infer(padded);

    // Map boxes back to input_image's own coordinate space - callers of
    // process_image() never see square-space boxes.
    return rf_detr_trt_backend::utils::unletterbox_detections(detections_padded, info);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during inference: %s", e.what());
    return {};
  }
}

void RfDetrDetection::publish_detection_result(
  const rf_detr_trt_backend::Detections & detections,
  const std_msgs::msg::Header & header)
{
  // Field layout confirmed via `ros2 interface show vision_msgs/msg/Detection2D`
  // against the installed vision_msgs version.
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;
  msg.detections.reserve(detections.size());

  for (const auto & det : detections) {
    vision_msgs::msg::Detection2D d;
    d.header = header;

    // Detection box coordinates are xyxy in pixel space; Detection2D wants
    // center + size.
    const float cx = (det.box.x1 + det.box.x2) * 0.5f;
    const float cy = (det.box.y1 + det.box.y2) * 0.5f;

    d.bbox.center.position.x = cx;
    d.bbox.center.position.y = cy;
    d.bbox.center.theta = 0.0;
    d.bbox.size_x = det.box.width();
    d.bbox.size_y = det.box.height();

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = rf_detr_trt_backend::utils::coco_label(det.class_id);
    hyp.hypothesis.score = det.score;
    d.results.push_back(hyp);

    msg.detections.push_back(std::move(d));
  }

  detection_pub_->publish(msg);
}

void RfDetrDetection::publish_overlay_result(
  const cv::Mat & overlay,
  const std_msgs::msg::Header & header)
{
  auto overlay_msg = cv_bridge::CvImage(
    header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg();
  overlay_pub_->publish(*overlay_msg);
}

} // namespace rf_detr_detection

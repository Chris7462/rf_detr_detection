#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <opencv2/core.hpp>

#include "rf_detr_trt_backend/rf_detr_trt_backend.hpp"


namespace rf_detr_detection
{

namespace fs = std::filesystem;

class RfDetrDetection : public rclcpp::Node
{
public:
  RfDetrDetection();
  ~RfDetrDetection();

private:
  bool initialize_parameters();
  bool initialize_inferencer();
  void initialize_ros_components();
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void timer_callback();

  // Returns Detections in the ORIGINAL image's coordinates - the caller
  // never sees square-space boxes; letterbox/unletterbox happens inside.
  rf_detr_trt_backend::Detections process_image(const cv::Mat & input_image);

  void publish_detection_result(
    const rf_detr_trt_backend::Detections & detections,
    const std_msgs::msg::Header & header);
  void publish_overlay_result(
    const cv::Mat & overlay, const std_msgs::msg::Header & header);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::shared_ptr<rf_detr_trt_backend::RFDetrTrtBackend> detector_;

  std::string input_topic_;
  std::string output_topic_;
  std::string output_overlay_topic_;
  int queue_size_;
  double processing_frequency_;
  int max_processing_queue_size_;

  rf_detr_trt_backend::RFDetrTrtBackend::Config config_;
  fs::path engine_path_;
  std::string engine_filename_;

  // Model input geometry, read back from detector_ via input_height()/
  // input_width() immediately after construction in initialize_inferencer()
  // (RFDetrTrtBackend::Config no longer carries height/width - they are
  // resolved from the engine itself). process_image() uses these as the
  // target size for letterbox_square().
  int input_height_;
  int input_width_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> img_buff_;
  std::mutex mtx_;
  std::atomic<bool> processing_in_progress_;
};

}

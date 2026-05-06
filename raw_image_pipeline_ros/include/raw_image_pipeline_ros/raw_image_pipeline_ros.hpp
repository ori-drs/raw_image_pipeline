//
// Copyright (c) 2021-2023, ETH Zurich, Robotic Systems Lab, Matias Mattamala, Timon Homberger. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for details.
//

#pragma once

#include <cv_bridge/cv_bridge.hpp>
// #include <glog/logging.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/distortion_models.h>
// #include <std_srvs/Trigger.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

#include <raw_image_pipeline/raw_image_pipeline.hpp>

namespace raw_image_pipeline {
class RawImagePipelineRos {
 public:
  // Constructor & destructor
  RawImagePipelineRos(const rclcpp::Node::SharedPtr& nh, const rclcpp::Node::SharedPtr& nh_private);
  ~RawImagePipelineRos();

  // Starts the node
  bool run();

 private:
  // Setup methods
  void loadParams();
  void setupRos();

  // Main callback method
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image);

  // Publishers
  void publishColorImage(const cv_bridge::CvImagePtr& cv_ptr_processed,                                // Processed image
                         const sensor_msgs::msg::Image::ConstSharedPtr& orig_image,                                 // Original image
                         const cv::Mat& mask,                                                          // Mask
                         int image_height, int image_width,                                            // Dimensions
                         const std::string& distortion_model, const cv::Mat& distortion_coefficients,  // Distortion
                         const cv::Mat& camera_matrix, const cv::Mat& rectification_matrix, const cv::Mat& projection_matrix,  //
                         image_transport::CameraPublisher& camera_publisher, image_transport::Publisher& slow_publisher,       //
                         int& skipped_images);

  // Services
  // ROS2HACK
  // bool resetWhiteBalanceHandler(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // Helpers
  std::string getTransportHintFromTopic(std::string& image_topic);

  void readRequiredParameter(const std::string& param, std::string& value) {
    nh_private_->declare_parameter(param, rclcpp::PARAMETER_STRING);
    if (!nh_private_->get_parameter(param, value)) {
      RCLCPP_FATAL_STREAM(nh_->get_logger(), "Could not get [" << param << "]");
      std::exit(-1);
    } else {
      RCLCPP_INFO_STREAM(nh_->get_logger(), param << ": " << value);
    }
  }

// string, bool, double, int, vector<double>
  // Note: we can't use templates for ROS2 since params are strongly typed
  template<class T>
  T readParameter(const std::string& param, T default_value) {
    nh_private_->declare_parameter(param, default_value);
    T value;
    if (!nh_private_->get_parameter(param, value)) {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "could not get [" << param << "], defaulting to: " << value);
    } else {
      RCLCPP_INFO_STREAM(nh_->get_logger(), param << ": " << value);
    }
    return value;
  }

  std::vector<double> readParameter(const std::string& param, std::vector<double> default_value);

  // ROS
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr nh_private_;
  // ROS2HACK - replace with executor
  // ros::AsyncSpinner spinner_;

  // Subscribers
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber sub_raw_image_;

  // Debayered publisher
  image_transport::CameraPublisher pub_image_debayered_;
  image_transport::Publisher pub_image_debayered_slow_;

  // Debayered publisher
  image_transport::CameraPublisher pub_image_color_;
  image_transport::Publisher pub_image_color_slow_;

  // Rectified image publisher
  image_transport::CameraPublisher pub_image_rect_;
  image_transport::Publisher pub_image_rect_mask_;
  image_transport::Publisher pub_image_rect_slow_;

  // Services
  // ROS2HACK
  // ros::ServiceServer reset_wb_temporal_consistency_server_;

  // ROS Params
  std::string input_topic_;
  std::string input_type_;
  std::string output_prefix_;
  std::string transport_;

  std::string output_encoding_;
  std::string output_frame_;

  // Slow topic
  int skip_number_of_images_for_slow_topic_;
  int skipped_images_for_slow_topic_;
  int skipped_images_for_slow_topic_rect_;

  // Postprocessing pipeline
  std::unique_ptr<RawImagePipeline> raw_image_pipeline_;
};

}  // namespace raw_image_pipeline

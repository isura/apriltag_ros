/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/single_image_detector.h"

#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

namespace apriltag_ros
{

SingleImageDetector::SingleImageDetector(rclcpp::Node::SharedPtr& node):
    tag_detector_(node)
{
  node_ = node;

  // Advertise the single image analysis service
  single_image_analysis_service_ =
      node_->create_service<apriltag_msgs::srv::AnalyzeSingleImage>("single_image_tag_detection",
        std::bind(&SingleImageDetector::analyzeImage, this, std::placeholders::_1, std::placeholders::_2));


  tag_detections_publisher_  =
      node_->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("tag_detections", 1);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Ready to do tag detection on single images");
}

bool SingleImageDetector::analyzeImage(
  const std::shared_ptr<apriltag_msgs::srv::AnalyzeSingleImage::Request>  request,
        std::shared_ptr<apriltag_msgs::srv::AnalyzeSingleImage::Response> response)
{

  RCLCPP_INFO(node_->get_logger(), "[ Summoned to analyze image ]");
  RCLCPP_INFO(node_->get_logger(), "Image load path: %s",
           request->full_path_where_to_get_image.c_str());
  RCLCPP_INFO(node_->get_logger(), "Image save path: %s",
           request->full_path_where_to_save_image.c_str());

  // Read the image
  cv::Mat image = cv::imread(request->full_path_where_to_get_image,
                             cv::IMREAD_COLOR);
  if (image.data == NULL)
  {
    // Cannot read image
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not read image " <<
                     request->full_path_where_to_get_image.c_str());
    return false;
  }

  // Detect tags in the image
  cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::msg::Header(),
                                                            "bgr8", image));
  loaded_image->header.frame_id = "camera";
  response->tag_detections =
      tag_detector_.detectTags(loaded_image,sensor_msgs::msg::CameraInfo::SharedPtr(
          new sensor_msgs::msg::CameraInfo(request->camera_info)));

  // Publish detected tags (AprilTagDetectionArray, basically an array of
  // geometry_msgs/PoseWithCovarianceStamped)
  tag_detections_publisher_->publish(response->tag_detections);

  // Save tag detections image
  tag_detector_.drawDetections(loaded_image);
  cv::imwrite(request->full_path_where_to_save_image, loaded_image->image);

  RCLCPP_INFO(node_->get_logger(), "Done!\n");

  return true;
}

} // namespace apriltag_ros

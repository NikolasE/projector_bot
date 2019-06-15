#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>

class ImageScreen
{
public:
  
  ImageScreen();
  
  void set_black();
  void show_image(const cv::Mat& img);
  void close_image_window();

  ros::Time last_image_callback_;
  ros::Subscriber img_sub_;

  std::vector<cv::Point2f> corners;
  void corner_trigger();

  void write_corners_();
  void read_corners_();
  std::string corner_file_path_;

  laser_geometry::LaserProjection projector_;

    std::string new_frame;
    std::vector<cv::Point2f> metric_corners_;


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;


  cv::Mat metric2Pixels;

//  cv::Mat warpMatrix;

  ros::Subscriber sub_laser_, sub_path;

  void path_cb(const nav_msgs::PathConstPtr& path);

  void laser_cb(const sensor_msgs::LaserScanConstPtr& msg);

  void point_cb_(const geometry_msgs::PointStampedConstPtr& pt);
  ros::Subscriber sub_pt;

  ros::Subscriber path_sub_;
  void path_cb(const nav_msgs::Path& path);
  void getPath();

// private:
  ros::NodeHandle nh_private_;
  ros::Subscriber toggle_sub_;
  ros::Subscriber black_sub_;
  std::string display_name_;

  bool debug_;
  bool window_is_open_;
  cv::Size img_size_;
  cv::Mat black_img_;
    cv::Mat output_img_;
  
  void black_cb_(const std_msgs::EmptyConstPtr& msg);
  void toggle_cb_(const std_msgs::BoolConstPtr& msg);
  void open_image_window();
  void img_cb_(const sensor_msgs::ImageConstPtr& msg);
  
  void reconnect();
};

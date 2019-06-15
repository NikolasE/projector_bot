#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "image_screen/ImageScreen.h"

#include <iostream>
#include <fstream>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::vector;
using cv::Point2f;

ImageScreen::ImageScreen():
nh_private_("~"),
display_name_("projection_window"),
debug_(false),
window_is_open_(false),
corner_file_path_("/tmp/tetris_corners.txt"),
tfListener(tfBuffer)
{
  nh_private_.param<int>("/projector_width", img_size_.width, 1920);
  nh_private_.param<int>("/projector_height", img_size_.height, 1080);
  nh_private_.param<bool>("/debug", debug_, false);
  if (debug_)
  {
    ROS_INFO("Running in debug mode");
  }

  ROS_INFO("Expecting projector size of  %i %i", img_size_.width, img_size_.height);
  
  black_img_ = cv::Mat(img_size_.height, img_size_.width, CV_8UC1);
  black_img_.setTo(0);
  
  reconnect();

  open_image_window();
  cv::imshow(display_name_, black_img_);
  cv::waitKey(1);
}

void ImageScreen::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    return;

    if (metric_corners_.size() != 4)
    {
        ROS_WARN("Metric corners: %zu", metric_corners_.size());
        return;
    }

    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*msg, cloud);


    vector<Point2f> srcPoints;

    for (size_t i=0; i<cloud.points.size(); i+= 10)
    {
        const auto& p = cloud.points[i];
        srcPoints.push_back(Point2f(p.x, p.y));
    }

    vector<Point2f> dstPoints;
    cv::perspectiveTransform(srcPoints, dstPoints, metric2Pixels);

    cv::Mat img = cv::Mat(img_size_.height, img_size_.width, CV_8UC3);
    img.setTo(cv::Scalar(255,0,0));

    for (const auto& px: dstPoints)
    {
        const cv::Point q(px.x, px.y);
        if (q.x < 0 || q.y < 0 || q.x >= img_size_.width || q.y >=img_size_.height) {
            continue;
        }

        cv::circle(img, q, 20, cv::Scalar(0, 0, 255), -1);

    }

    cv::imshow(display_name_, img);
    cv::waitKey(1);
}


void ImageScreen::point_cb_(const geometry_msgs::PointStampedConstPtr &pt_s) {

    if (metric_corners_.size() != 4)
    {
        ROS_WARN("Metric corners: %zu", metric_corners_.size());
        return;
    }

    cv::Mat img = cv::Mat(img_size_.height, img_size_.width, CV_8UC3);
    img.setTo(cv::Scalar(255,0,0));

    geometry_msgs::PointStamped p2;

    std::string new_frame = "laser";

    ROS_INFO("Transforming from %s to %s", pt_s->header.frame_id.c_str(), new_frame.c_str());

    try {
        tfBuffer.transform(*pt_s, p2, new_frame);
        ROS_INFO("In laser Frame: %f %f", p2.point.x, p2.point.y);
    }
    catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
            return;
     }

    geometry_msgs::Point pt = p2.point;

//    {
//        vector<cv::Point2f> pts;
//
//        float l = 0.5;
//
//        pts.push_back(Point2f(pt.x, pt.y));
//        pts.push_back(Point2f(pt.x+l, pt.y));
//        pts.push_back(Point2f(pt.x+l, pt.y+l));
//        pts.push_back(Point2f(pt.x, pt.y+l));
//
//        vector<Point2f> dstPoints;
//        cv::perspectiveTransform(pts, dstPoints, metric2Pixels);
//
//        for (int i=0; i<4; ++i)
//        {
//            const auto& p1 = dstPoints[i];
//            const auto& p2 = dstPoints[(i+1)%4];
//
//            cv::line(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(255, 255, 0), 30);
//        }
//
//    }


    vector<Point2f> dstPoints, srcPoints;
    srcPoints.push_back(Point2f(pt.x, pt.y));
    cv::perspectiveTransform(srcPoints, dstPoints, metric2Pixels);

    Point2f d  = dstPoints[0];

    ROS_INFO("Transformed to %f %f", d.x, d.y);

    cv::circle(img, cv::Point(d.x, d.y), 20, cv::Scalar(0, 255, 0), -1);

    show_image(img);
}


void ImageScreen::reconnect()
{

    read_corners_();

  last_image_callback_ = ros::Time::now();

  sub_pt = nh_private_.subscribe("/pt", 1, &ImageScreen::point_cb_, this);
  sub_laser_ = nh_private_.subscribe("/scan", 1, &ImageScreen::laser_cb, this);

  img_sub_ = nh_private_.subscribe("/usb_cam/image_raw", 1, &ImageScreen::img_cb_, this);

  toggle_sub_ = nh_private_.subscribe("activate", 1, &ImageScreen::toggle_cb_, this);
  toggle_sub_ = nh_private_.subscribe("black", 1, &ImageScreen::black_cb_, this);
}



void ImageScreen::read_corners_() {
    {
        std::ifstream infile(corner_file_path_);
        corners.clear();
        float a, b;
        while (infile >> a >> b) {
            corners.push_back(cv::Point2f(a, b));
        }
        corner_trigger();
    }

    {
        std::string filename = "/tmp/points.txt";
        std::ifstream infile(filename);
        float a, b;

        metric_corners_.clear();

        while (infile >> a >> b) {
            ROS_INFO("%f %f", a, b);
            metric_corners_.push_back(cv::Point2f(a, b));
        }

        if (metric_corners_.size() != 4)
        {
            ROS_FATAL("metric size from file: %zu", metric_corners_.size());
            return;
        }

        int w = img_size_.width;
        int h = img_size_.height;
        std::vector<cv::Point2f> pixels;
        pixels.push_back(cv::Point2f(0,0));
        pixels.push_back(cv::Point2f(w,0));
        pixels.push_back(cv::Point2f(w,h));
        pixels.push_back(cv::Point2f(0,h));

        ROS_INFO("Created metric2pixels");

        metric2Pixels = cv::getPerspectiveTransform(metric_corners_, pixels);
    }
}

void ImageScreen::corner_trigger()

{
  cv::Mat img = cv::Mat(img_size_.height, img_size_.width, CV_8UC3);
  img.setTo(cv::Scalar(255,0,0));

  if (corners.size() == 4)
  {
    std::string cat_path(std::getenv("HOME"));
    cat_path += "/catkin_ws/src/projector/cat.jpeg";
    cv::Mat cat_img = cv::imread(cat_path);
    if (cat_img.empty())
    {
      ROS_WARN("Could not find test image at '%s'", cat_path.c_str());
      return;
    }

    int w = cat_img.cols;
    int h = cat_img.rows;
    std::vector<cv::Point2f> input;
    input.push_back(cv::Point2f(0,0));
    input.push_back(cv::Point2f(w,0));
    input.push_back(cv::Point2f(w,h));
    input.push_back(cv::Point2f(0,h));

    write_corners_();

    cv::Mat warpMatrix = cv::getPerspectiveTransform(input, corners);
    cv::warpPerspective(cat_img, img, warpMatrix, img_size_);
  }

  for (int i = 0; i < corners.size(); ++i) {
    cv::circle(img, corners[i], 20, cv::Scalar(0,0,255), 3);
  }

  for (int i=0; i<corners.size(); ++i)
  {
    cv::line(img, corners[i], corners[(i+1)%corners.size()], cv::Scalar(0, 255, 0));
  }

  show_image(img);

//  cv::imshow(display_name_, img);
//  cv::waitKey(1);
}


void ImageScreen::open_image_window()
{
  if (window_is_open_)
    return;
  window_is_open_ = true;
  
  ROS_INFO("Opening window");
  cv::namedWindow(display_name_, CV_WINDOW_NORMAL); // CV_WINDOW_AUTOSIZE);

//  cv::setMouseCallback(display_name_, mouseCB, this);

//  if (!debug_)
  {
    cv::moveWindow(display_name_, 0,0); // 1920*1.5, 100);
    cv::setWindowProperty(display_name_, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  }
  cv::waitKey(10);
}

void ImageScreen::close_image_window()
{
  if (!window_is_open_)
    return;
  
  window_is_open_ = false;
  ROS_INFO("Closing image");
  cv::destroyWindow(display_name_);
}

void ImageScreen::show_image(const cv::Mat& img)
{
  open_image_window();
  cv::imshow(display_name_, img);
  cv::waitKey(2);
}

void ImageScreen::set_black()
{
  show_image(black_img_);
}



void ImageScreen::img_cb_(const sensor_msgs::ImageConstPtr& msg)
{
  last_image_callback_ = ros::Time::now();
  // ROS_INFO_STREAM("img_cb_ reached");
  
  cv_bridge::CvImageConstPtr cv_ptr;
  
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    //        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  /// compute warp matrix (todo: only if input size changed)
  const cv::Mat& img = cv_ptr->image;

  if (corners.size() == 4)
  {
    int w = img.cols;
    int h = img.rows;
    std::vector<cv::Point2f> input;
    input.push_back(cv::Point2f(0,0));
    input.push_back(cv::Point2f(w,0));
    input.push_back(cv::Point2f(w,h));
    input.push_back(cv::Point2f(0,h));

    cv::Mat warpMatrix = cv::getPerspectiveTransform(input, corners);

    black_img_.copyTo(output_img_);

    cv::warpPerspective(img, output_img_, warpMatrix, img_size_);
    show_image(output_img_);
  }else
  {
    cv::Mat resized;
    cv::resize(img, resized, img_size_);
    show_image(resized);
  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_screen");

  ImageScreen screen;
  while(ros::ok())
  {
    auto now = ros::Time::now();
//    if(screen.last_image_callback_ != ros::Time(0) && now - screen.last_image_callback_ > ros::Duration(2))
//    {
//      ROS_FATAL_STREAM("Last image callback was at " << screen.last_image_callback_ << ", killing");
//      return EXIT_FAILURE;
//    }
    ros::spinOnce();
    ros::Rate(10.0).sleep();
    cv::waitKey(1);
  }
  
  ros::spin();
  return EXIT_SUCCESS;
}



void ImageScreen::black_cb_(const std_msgs::EmptyConstPtr& msg)
{
    ROS_INFO("Going dark");
    set_black();
}

void ImageScreen::toggle_cb_(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data)
    {
        open_image_window();
    }else
    {
        close_image_window();
    }
}




void ImageScreen::write_corners_()
{
    if (corners.size() != 4)
    {
        ROS_INFO("Not writing current corners (size is %zu)", corners.size());
        return;
    }

    std::ofstream myfile;
    myfile.open (corner_file_path_);
    for (int i=0; i<corners.size(); ++i)
    {
        myfile << corners[i].x << "  " << corners[i].y << std::endl;
    }

    myfile.close();
}

void mouseCB(int event, int x, int y, int flags, void* userdata)
{
//ROS_INFO("%i %i", x, y);
//  ImageScreen* is = (ImageScreen*) userdata;
//
//  if  ( event == cv::EVENT_LBUTTONUP )
//  {
//    if (is->corners.size() < 4)
//    {
//      is->corners.push_back(cv::Point2f(x, y));
//    } else {
//      // update closest point
//      float min_dist = 1e6;
//      int best_ndx = -1;
//
//      for (int i=0; i<is->corners.size(); ++i)
//      {
//        const cv::Point2f& p = is->corners[i];
//        float d = abs(p.x-x)+abs(p.y-y);
//        if (d < min_dist || best_ndx < 0)
//        {
//          best_ndx = i;
//          min_dist = d;
//        }
//      }
//
//      is->corners[best_ndx] = cv::Point2f(x, y);
//    }
//    is->corner_trigger();
//  }
}
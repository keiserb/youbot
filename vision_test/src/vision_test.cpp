/*
 * vision_test.cpp
 *
 *  Created on: Jun 26, 2013
 *      Author: keiserb
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
static const char WINDOW2[] = "Depth window";
static const char WINDOW3[] = "Edges";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub;
  image_transport::Publisher image_pub_;
  ros::Publisher pose_pub;

public:
  ImageConverter() :
      it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    depth_sub = it_.subscribe("/camera/depth_registered/image", 1, &ImageConverter::depthCb, this);
    pose_pub = nh_.advertise<geometry_msgs::Pose2D>("/Pose",1);

    cv::namedWindow(WINDOW);
    cv::namedWindow(WINDOW2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOW2);
  }

  std::vector<geometry_msgs::Pose2D> findCandidates(cv::Mat & image)
  {
    std::vector<geometry_msgs::Pose2D> candidates;
    cv::Mat image_gr;
    //Create Grayscale image
    //cv::cvtColor(image, image_gr, CV_RGB2GRAY);
    //blur image
    cv::blur(image, image_gr, cv::Size(5, 5));
    //find edges
    cv::Canny(image_gr, image_gr, 150, 200);

    cv::imshow(WINDOW3, image_gr);
    cv::waitKey(3);
    // find contours
    std::vector<std::vector<cv::Point> > contours_initial;
    cv::findContours(image_gr, contours_initial, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    //find contours which are closed and join them
    std::vector<std::vector<cv::Point> > contours;
    for (int i = 0; i < (int)contours_initial.size(); i++)
    {
      bool close_point = false;
      int close_contour;
      for (int j = 0; j < (int)contours_initial[i].size(); j++)
      {
        for (int k = i + 1; k < (int)contours_initial.size(); k++)
        {
          for (int l = 0; l < (int)contours_initial[k].size(); l++)
          {
            double dist = (contours_initial[i][j].x - contours_initial[k][l].x)
                * (contours_initial[i][j].x - contours_initial[k][l].x)
                + (contours_initial[i][j].y - contours_initial[k][l].y)
                    * (contours_initial[i][j].y - contours_initial[k][l].y);
            if (dist < 20 * 20)
            {
              close_point = true;
              close_contour = k;
            }
          }
          if (close_point)
            break;
        }
        if (close_point)
          break;
      }
      // if close point, put all points from i to close_contour
      if (close_point)
      {
        for (int j = 0; j < (int)contours_initial[i].size(); j++)
        {
          contours_initial[close_contour].push_back(contours_initial[i][j]);
        }
      }
      else
      {
        contours.push_back(contours_initial[i]);
      }
    }

    // get convex hull and enclosing circle
    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());

    for (int i = 0; i < (int)contours.size(); i++)
    {
      cv::convexHull(cv::Mat(contours[i]), contours_poly[i]);
      cv::minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
    }

    for (int i = 0; i < (int)contours.size(); i++)
    {
      // is this circle large enough?
      if (radius[i] > 5)
      {
        // is the circle contained in another, bigger circle?
        bool circle_ok = true;
        for (int j = 0; j < (int)contours.size(); j++)
        {
          if (i != j)
          {
            int ri = (int)radius[i];
            int rj = (int)radius[j];
            if (ri < rj)
            {
              if ((center[i].x - center[j].x) * (center[i].x - center[j].x)
                  + (center[i].y - center[j].y) * (center[i].y - center[j].y) < rj * rj)
                circle_ok = false;
            }
          }
        }
        if (circle_ok && radius[i] > 12 && radius[i] < 100)
        {
          if (center[i].y < 430)
          {

            // Get the moment
            cv::Moments mu = cv::moments(contours_poly[i], false);

            //  Get the mass center
            cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

            // get both eigenvalues
            double lambda_part_1 = 0.5 * (mu.mu20 + mu.mu02) / mu.m00;
            double lambda_part_2 = 0.5
                * sqrt(
                    4 * mu.mu11 * mu.mu11 / (mu.m00 * mu.m00)
                        + (mu.mu20 / mu.m00 - mu.mu02 / mu.m00) * (mu.mu20 / mu.m00 - mu.mu02 / mu.m00));
            double lambda1 = lambda_part_1 + lambda_part_2;
            double lambda2 = lambda_part_1 - lambda_part_2;

            double scale_ratio = sqrt(lambda1 / lambda2);

            if (scale_ratio > 1.5)
            {
              double theta = 0.5 * atan2(2 * mu.mu11 / mu.m00, mu.mu20 / mu.m00 - mu.mu02 / mu.m00);

              geometry_msgs::Pose2D pos;
              pos.x = mc.x; //atan((mc.x - camera_cx)/camera_fx);
              pos.y = mc.y; //atan((mc.y - camera_cy)/camera_fy);
              pos.theta = theta;
              candidates.push_back(pos);

              // draw
              cv::Point2f v = cv::Point2f(radius[i] * cos(theta), radius[i] * sin(theta));

              cv::drawContours(image, contours_poly, i, cv::Scalar(0, 0, 255), 1, 8, std::vector<cv::Vec4i>(), 0,
                               cv::Point());

              cv::circle(image, mc, 4, cv::Scalar(255, 0, 0), -1, 8, 0);
              cv::line(image, mc + v, mc - v, cv::Scalar(255, 0, 0), 1);
            }
          }
        }
      }
    }

    return candidates;
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow(WINDOW2, cv_ptr->image);
    cv::waitKey(3);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<geometry_msgs::Pose2D> pose_vect;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    pose_vect = findCandidates(cv_ptr->image);
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    while(!pose_vect.empty())
    {
      pose_pub.publish(pose_vect.back());
      pose_vect.pop_back();
    }
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


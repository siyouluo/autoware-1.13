/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "autoware_msgs/PointsImage.h"
#include "autoware_msgs/ProjectionMatrix.h"
//#include "autoware_msgs/CameraExtrinsic.h"


#include <sensor_msgs/PointCloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <include/points_image/points_image.hpp>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

//几个常数矩阵
cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation vector
cv::Mat rMat = cv::Mat::eye(3, 3, CV_64FC1);
cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // Translation vector

static cv::Mat cameraExtrinsicMat;
static cv::Mat cameraMat;
static cv::Mat distCoeff;
static cv::Size imageSize;

static ros::Publisher pub;
ros::Publisher _pub_rgb_cloud;

static void projection_callback(const autoware_msgs::ProjectionMatrix& msg)
{
  cameraExtrinsicMat = cv::Mat(4, 4, CV_64F);
  for (int row = 0; row < 4; row++)
  {
    for (int col = 0; col < 4; col++)
    {
      cameraExtrinsicMat.at<double>(row, col) = msg.projection_matrix[row * 4 + col];
    }
  }
  resetMatrix();
}

static void intrinsic_callback(const sensor_msgs::CameraInfo& msg)
{
  imageSize.height = msg.height;
  imageSize.width = msg.width;

  cameraMat = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      cameraMat.at<double>(row, col) = msg.K[row * 3 + col];
    }
  }

  distCoeff = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distCoeff.at<double>(col) = msg.D[col];
  }
  resetMatrix();
}

static void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (cameraExtrinsicMat.empty() || cameraMat.empty() || distCoeff.empty() || imageSize.height == 0 ||
      imageSize.width == 0)
  {
    ROS_INFO("[points2image]Looks like camera_info or projection_matrix are not being published.. Please check that "
             "both are running..");
    return;
  }

  autoware_msgs::PointsImage pub_msg = pointcloud2_to_image(msg, cameraExtrinsicMat, cameraMat, distCoeff, imageSize);
  pub.publish(pub_msg);
}


// 给定一张图片、一个点云、相机内参(包括畸变系数)、雷达到相机的坐标变化，将像素的RGB赋给点云，得到RGB点云返回.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr image2pointcloud_fusion(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
    cv::Mat input_image,
    cv::Mat cameraExtrinsicMat, 
    cv::Mat cameraMat, 
    cv::Mat distCoeff, 
    cv::Size imageSize)
{
  Eigen::Matrix4d cam2lidar_transform;
  cam2lidar_transform << cameraExtrinsicMat.at<double>(0, 0), cameraExtrinsicMat.at<double>(0,1), cameraExtrinsicMat.at<double>(0,2), cameraExtrinsicMat.at<double>(0,3), 
                        cameraExtrinsicMat.at<double>(1, 0), cameraExtrinsicMat.at<double>(1,1), cameraExtrinsicMat.at<double>(1,2), cameraExtrinsicMat.at<double>(1,3), 
                        cameraExtrinsicMat.at<double>(2, 0), cameraExtrinsicMat.at<double>(2,1), cameraExtrinsicMat.at<double>(2,2), cameraExtrinsicMat.at<double>(2,3),
                        cameraExtrinsicMat.at<double>(3, 0), cameraExtrinsicMat.at<double>(3,1), cameraExtrinsicMat.at<double>(3,2), cameraExtrinsicMat.at<double>(3,3);
  Eigen::Matrix4d lidar2cam_transform = cam2lidar_transform.inverse();
  if (input_cloud_ptr->size() == 0)
  {

    ROS_WARN("input cloud is empty, please check it out!");
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // transform lidar points from lidar coordinate to camera coordiante
  pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud, lidar2cam_transform); // lidar coordinate(forward x+, left y+, up z+)
                                                                                       // front camera coordiante(right x+, down y+, forward z+) (3D-3D)
                                                                                       // using the extrinsic matrix between this two coordinate system
  std::vector<cv::Point3d> lidar_points;
  std::vector<float> intensity;
  std::vector<cv::Point2d> imagePoints;

  // reserve the points in front of the camera(z>0)
  for (int i = 0; i <= (int)transformed_cloud->points.size(); i++)
  {
    if (transformed_cloud->points[i].z > 0)
    {
      lidar_points.push_back(cv::Point3d(transformed_cloud->points[i].x, transformed_cloud->points[i].y, transformed_cloud->points[i].z));
      // intensity.push_back(transformed_cloud->points[i].intensity);
    }
  }

  // project lidar points from the camera coordinate to the image coordinate(right x+, down y+)
  cv::projectPoints(lidar_points, rMat, tVec, cameraMat, distCoeff, imagePoints);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_transback(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < (int)imagePoints.size(); i++)
  {
    if (imagePoints[i].x >= 0 && imagePoints[i].x < imageSize.width && imagePoints[i].y >= 0 && imagePoints[i].y < imageSize.height)
    {
      pcl::PointXYZRGB point;          // reserve the lidar points in the range of image
      point.x = lidar_points[i].x; // use 3D lidar points and RGB value of the corresponding pixels
      point.y = lidar_points[i].y; // to create colored point clouds
      point.z = lidar_points[i].z;
      point.r = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[2];
      point.g = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[1];
      point.b = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[0];
      // point.i = intensity[i];
      colored_cloud->points.push_back(point);
    }
  }
  // transform colored points from camera coordinate to lidar coordinate
  // pcl::transformPointCloud(*colored_cloud, *colored_cloud_transback, cam2vehicle_transform);
  pcl::transformPointCloud(*colored_cloud, *colored_cloud_transback, cam2lidar_transform);
  return colored_cloud_transback;
}


static void image2pointcloud_fusion_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& image_msg)
{
  if (cameraExtrinsicMat.empty() || cameraMat.empty() || distCoeff.empty() || imageSize.height == 0 ||
      imageSize.width == 0)
  {
    ROS_INFO("[points2image]Looks like camera_info or projection_matrix are not being published.. Please check that "
             "both are running..");
    return;
  }

  std::string stamp_a = std::to_string(cloud_msg->header.stamp.toSec());
  std::string stamp_b = std::to_string(image_msg->header.stamp.toSec());
  ROS_INFO("fusion cloud (@[%s]) & image (@[%s])", stamp_a.c_str(), stamp_b.c_str());

  // sensor_msgs to cv image
  cv_bridge::CvImagePtr cv_image_ptr;
  try
  {
    cv_image_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception e)
  {
    ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
    return;
  }
  cv::Mat image = cv_image_ptr->image;

  // sensor_msgs to pointxyz
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *input_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_transback = image2pointcloud_fusion(input_cloud_ptr, image, cameraExtrinsicMat, cameraMat, distCoeff, imageSize);
  sensor_msgs::PointCloud2 cloud_rgb_msg;
  pcl::toROSMsg(*colored_cloud_transback, cloud_rgb_msg);
  cloud_rgb_msg.header = cloud_msg->header;
  _pub_rgb_cloud.publish(cloud_rgb_msg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "points2image");
  ros::NodeHandle n;

  ros::NodeHandle private_nh("~");

  std::string camera_info_topic_str;
  std::string projection_matrix_topic;
  std::string pub_topic_str = "/points_image";

  private_nh.param<std::string>("projection_matrix_topic", projection_matrix_topic, "/projection_matrix");
  private_nh.param<std::string>("camera_info_topic", camera_info_topic_str, "/camera_info");


  std::string name_space_str = ros::this_node::getNamespace();

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      /* if name space obtained by ros::this::node::getNamespace()
         starts with "//", delete one of them */
      name_space_str.erase(name_space_str.begin());
    }
    pub_topic_str = name_space_str + pub_topic_str;
    projection_matrix_topic = name_space_str + projection_matrix_topic;
    camera_info_topic_str = name_space_str + camera_info_topic_str;
  }

  std::string points_topic;
  if (private_nh.getParam("points_node", points_topic))
  {
    ROS_INFO("[points2image]Setting points node to %s", points_topic.c_str());
  }
  else
  {
    ROS_INFO("[points2image]No points node received, defaulting to points_raw, you can use _points_node:=YOUR_TOPIC");
    points_topic = "/points_raw";
  }
  std::string image_topic_src_str;
  if (private_nh.getParam("image_topic_src", image_topic_src_str))
  {
    ROS_INFO("[points2image]Setting image topic to %s", image_topic_src_str.c_str());
  }
  else
  {
    ROS_INFO("[points2image]No image_topic_src received, defaulting to image_raw.");
    image_topic_src_str = "/image_raw";
  }

  ROS_INFO("[points2image]Publishing to... %s", pub_topic_str.c_str());
  pub = n.advertise<autoware_msgs::PointsImage>(pub_topic_str, 10);

  ros::Subscriber sub = n.subscribe(points_topic, 1, callback);

  ROS_INFO("[points2image]Subscribing to... %s", projection_matrix_topic.c_str());
  ros::Subscriber projection = n.subscribe(projection_matrix_topic, 1, projection_callback);
  ROS_INFO("[points2image]Subscribing to... %s", camera_info_topic_str.c_str());
  ros::Subscriber intrinsic = n.subscribe(camera_info_topic_str, 1, intrinsic_callback);

  _pub_rgb_cloud = n.advertise<sensor_msgs::PointCloud2>("/points_rgb", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, points_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, image_topic_src_str, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> image2cloud_SyncPolicy;
  message_filters::Synchronizer<image2cloud_SyncPolicy> sync(image2cloud_SyncPolicy(10), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&image2pointcloud_fusion_callback, _1, _2));
  ros::spin();
  return 0;
}

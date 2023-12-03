// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <math.h>
#include <vector>
#include <aloam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <fstream>
#include <stdlib.h>
#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <sys/select.h>

/*직접 추가Start---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>



//**********************      
#define DEPTHRANGE 0.3  // 0.1 = +-10cm = 20cm, 0.2 = +-20cm = 40cm  0.05 = +-5cm = 10cm
//#define ZEDRESOLW 1280 // HD2K: 2208 , HD1080: 1920, HD720: 1280
//#define ZEDRESOLH 720 // HD2K: 1242 , HD1080: 1080, HD720: 720

#define LIMIT_POINTSIZE 1000000
  
bool _newLeftcamInfo = false;   // 새 카메라 내부 파라미터가 들어왔는지 확인하는 flag.
bool _newLeftImg = false;
bool _newDepthImg = false;
bool _newZedPose = false;
bool ros_status=false;

//color pcd 저장용
//pcl::PointCloud<pcl::PointXYZRGB> _laserCloudFullResColor;
//pcl::PointCloud<pcl::PointXYZI> _laserCloudFullResColor;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr _laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr _ColorPointAccum(new pcl::PointCloud<pcl::PointXYZRGB>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZI>());

//intrinsic matrix 초기화
cv::Mat K = (cv::Mat_<float>(3,3) <<   0, 0, 0,  
                                       0, 0, 0, 
                                       0, 0, 0);

//extrinsic 초기화 및 선언 (translation unit --> meter)
cv::Mat E = (cv::Mat_<float>(3,4) << 					
		    -1,  0,  0,  0.06, //0.06 ,0.15, 0.165
                     0,  0, -1, -0.0354, //-0.056, -0.026, 0.066
                     0,  -1, 0, -0.092); //0.0444

cv::Mat D = (cv::Mat_<float>(1,5) << 0, 0, 0, 0, 0);
cv::Mat Pose= (cv::Mat_<double>(1,7) << 0, 0, 0, 0, 0, 0, 0);

//camera matrix(projection matrix)
cv::Mat KE;

//float* depths;
//cv::Mat mat_depth;

cv::Mat _mat_left;



//***********************
std::ofstream cam_info("/root/catkin_ws/Dataset/ADOP_dataset/zed2i_info.txt", std::ios_base::out);
std::ofstream fout_masks ("/root/catkin_ws/Dataset/ADOP_dataset/images.txt", std::ios_base::out);
std::ofstream fout_poses ("/root/catkin_ws/Dataset/ADOP_dataset/poses.txt", std::ios_base::out);


int img_width=0, img_height=0;
int filecounter=0;
int imgcounter=0;
int frameCount = 0;
int getPcd =-1;
int getImage = -1;
/*직접 추가End---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;








const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851








int laserCloudValidInd[125];
int laserCloudSurroundInd[125];




// input: from odom
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());




// ouput: all visualble cube points
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());




// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());




//input & output: points in one frame. local --> global
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());




// points in every cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];




//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());




double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);




// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);




Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);








std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mBuf;




pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;




std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;




PointType pointOri, pointSel;




ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath, pubColorpointAccum;




nav_msgs::Path laserAfterMappedPath;




// set initial guess
void transformAssociateToMap()
{
  q_w_curr = q_wmap_wodom * q_wodom_curr;
  t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}




void transformUpdate()
{
  q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
  t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}




void pointAssociateToMap(PointType const *const pi, PointType *const po)
{ //local --> world coordinate
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
  //po->intensity = 1.0;
}

/*직접 추가Start---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//point의 local 좌표를 world좌표로 변환
void pointAssociateToMapForColor(pcl::PointXYZRGB const *const pi, pcl::PointXYZRGB *const po)
{
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
}
//point의 world 좌표를 camerad의 world좌표로 변환 (LiDAR World좌표계와 Camera World좌표계가 존재하는 이유: ROS기반 A-loam(LiDAR)와 Visual SLAM(ZED Stereo Camera)를 작동시키기 때문)
void mappedPointToCameraWorld(pcl::PointXYZRGB const *const pi, pcl::PointXYZRGB *const po){
	cv::Mat_<float> homo_3D_L(4,1); //x,y,z,1 of lidar
	cv::Mat_<float> homo_3D_C(3,1); //x,y,z of camera
	
	homo_3D_L.at<float>(0,0)=pi->x;
	homo_3D_L.at<float>(1,0)=pi->y;
	homo_3D_L.at<float>(2,0)=pi->z;
	homo_3D_L.at<float>(3,0)=1;

	homo_3D_C=E*homo_3D_L; //lidar coordinate to camera coordinate
	
	po->x=homo_3D_C.at<float>(0,0);
	po->y=homo_3D_C.at<float>(1,0);
	po->z=homo_3D_C.at<float>(2,0);
}
/*직접 추가End---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/	


void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{ //world --> local coordinate
  Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
  po->x = point_curr.x();
  po->y = point_curr.y();
  po->z = point_curr.z();
  //po->intensity = pi->intensity;
}



void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
  mBuf.lock();
  cornerLastBuf.push(laserCloudCornerLast2);
  mBuf.unlock();
}




void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
  mBuf.lock();
  surfLastBuf.push(laserCloudSurfLast2);
  mBuf.unlock();
}




void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
  mBuf.lock();
  fullResBuf.push(laserCloudFullRes2);
  mBuf.unlock();
}


/*직접 추가Start---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Zed Stereo Camera로부터 정보를 불러오기 위함
void imageLeftHandler(const sensor_msgs::Image::ConstPtr& msg) {
   //ROS_INFO("Left image received from ZED - Size: %dx%d", msg->width, msg->height);

  cv_bridge::CvImageConstPtr cv_ptr;
  try{
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
  ROS_ERROR("cv_bridge exception: %s", e.what());
  return;
  }

  _mat_left = cv_ptr->image;
   //cv::imshow("mat", _mat_left);
   //cv::waitKey(1000);
  _newLeftImg = true;
}

void zedPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& msg) {
   //printf("in zed pose handler!\n");
	double tx,ty,tz,ox,oy,oz,ow;
	
	tx = msg->pose.position.x;
	ty = msg->pose.position.y;
	tz = msg->pose.position.z;
	ox = msg->pose.orientation.x;
	oy = msg->pose.orientation.y;
	oz = msg->pose.orientation.z;
	ow = msg->pose.orientation.w;

   // Camera position in map frame
	Pose=(cv::Mat_<double>(1,7) << ox, oy, oz, ow, tx, ty, tz);
   
			
			
  // Output the measure
  //ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - OX: %.2f OY: %.2f OZ: %.2f OW: %.2f", msg->header.frame_id.c_str(), tx, ty, tz, ox, oy, oz, ow);
}

/*
void depthHandler(const sensor_msgs::Image::ConstPtr& msg) {


  //printf("in depth img handler!\n");


  cv_bridge::CvImageConstPtr cv_ptr;
  try{
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
  }


  mat_depth = cv_ptr->image;


   // Get a pointer to the depth values casting the data
  // pointer to floating point
  depths = (float*)(&msg->data[0]);


  _newDepthImg = true;


}*/


void leftcamInfoHandler(const sensor_msgs::CameraInfo::ConstPtr& msg) {
  if(K.at<float>(0,0) == 0 && _newLeftcamInfo == false){
    K = (cv::Mat_<float>(3,3) <<  msg->P[0], msg->P[1], msg->P[2],
                                msg->P[4], msg->P[5], msg->P[6],
                                msg->P[8], msg->P[9], msg->P[10] );
	img_width=msg->width;
	img_height=msg->height;
	D=(cv::Mat_<float>(1,5) << msg->D[0], msg->D[1], msg->D[2], msg->D[3], msg->D[4]); 

    ROS_INFO("\n fx: %f\n fy: %f\n cx: %f\n cy: %f\n size: %dx%d\n Distortion: %f, %f, %f, %f, %f\n",K.at<float>(0,0), K.at<float>(1,1), K.at<float>(0,2), K.at<float>(1,2), img_width, img_height, D.at<float>(0,0), D.at<float>(0,1), D.at<float>(0,2), D.at<float>(0,3), D.at<float>(0,4) );

    // save the information of zed2i in zed2i_info.txt
	cam_info << "width: " << img_width << std::endl << "height: " << img_height<<std::endl;
	cam_info << "fx: " << K.at<float>(0,0) << std::endl;
	cam_info << "fy: " << K.at<float>(1,1) << std::endl;
	cam_info << "cx: " << K.at<float>(0,2) << std::endl;
	cam_info << "cy: " << K.at<float>(1,2) << std::endl;
 	cam_info << "D: " << K.at<float>(0,0) <<" "<<K.at<float>(0,1) <<" "<< K.at<float>(0,2) <<" "<<K.at<float>(0,3) <<" "<<K.at<float>(0,4) <<" "<< std::endl;   

    // 두 행렬 곱하기.
    KE = K * E;
  }
  _newLeftcamInfo = true;   // 한번만 시행되도록 flag ON.
  cam_info.close();  
}

//zed*************
int CheckSTDIN(int a_timeout){
	fd_set rfds;
	struct timeval tv;

	FD_ZERO(&rfds);
	FD_SET(0, &rfds);
	tv.tv_sec=a_timeout/1000000;
	tv.tv_usec=a_timeout%1000000;
	
	return select(1, &rfds, NULL, NULL, &tv)>0;
}
/*직접 추가End---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

//receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
  mBuf.lock();
  odometryBuf.push(laserOdometry);
  mBuf.unlock();




  // high frequence publish
  Eigen::Quaterniond q_wodom_curr;
  Eigen::Vector3d t_wodom_curr;
  q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
  q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
  q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
  q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
  t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
  t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
  t_wodom_curr.z() = laserOdometry->pose.pose.position.z;




  Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
  Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;



  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "/camera_init";
  odomAftMapped.child_frame_id = "/aft_mapped";
  odomAftMapped.header.stamp = laserOdometry->header.stamp;
  odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
  odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
  odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
  odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
  odomAftMapped.pose.pose.position.x = t_w_curr.x();
  odomAftMapped.pose.pose.position.y = t_w_curr.y();
  odomAftMapped.pose.pose.position.z = t_w_curr.z();
  pubOdomAftMappedHighFrec.publish(odomAftMapped);
} 




void process()
{
  while(1)
  {
      while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
          !fullResBuf.empty() && !odometryBuf.empty())
      {
          mBuf.lock();
          while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
              odometryBuf.pop();
          if (odometryBuf.empty())
          {
              mBuf.unlock();
              break;
          }




          while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
              surfLastBuf.pop();
          if (surfLastBuf.empty())
          {
              mBuf.unlock();
              break;
          }




          while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
              fullResBuf.pop();
          if (fullResBuf.empty())
          {
              mBuf.unlock();
              break;
          }




          timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
          timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
          timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
          timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();




          if (timeLaserCloudCornerLast != timeLaserOdometry ||
              timeLaserCloudSurfLast != timeLaserOdometry ||
              timeLaserCloudFullRes != timeLaserOdometry)
          {
              printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
              printf("unsync messeage!");
              mBuf.unlock();
              break;
          }




          laserCloudCornerLast->clear();
          pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
          cornerLastBuf.pop();




          laserCloudSurfLast->clear();
          pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
          surfLastBuf.pop();




          laserCloudFullRes->clear();
          pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
        




          fullResBuf.pop();




          q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
          q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
          q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
          q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
          t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
          t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
          t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
          odometryBuf.pop();




          while(!cornerLastBuf.empty())
          {
              cornerLastBuf.pop();
              printf("drop lidar frame in mapping for real time performance \n");
          }




          mBuf.unlock();




          TicToc t_whole;




          transformAssociateToMap();




          TicToc t_shift;
          int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
          int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
          int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;




          if (t_w_curr.x() + 25.0 < 0)
              centerCubeI--;
          if (t_w_curr.y() + 25.0 < 0)
              centerCubeJ--;
          if (t_w_curr.z() + 25.0 < 0)
              centerCubeK--;




          while (centerCubeI < 3)
          {
              for (int j = 0; j < laserCloudHeight; j++)
              {
                  for (int k = 0; k < laserCloudDepth; k++)
                  {
                      int i = laserCloudWidth - 1;
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      for (; i >= 1; i--)
                      {
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      }
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeCornerPointer;
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeSurfPointer;
                      laserCloudCubeCornerPointer->clear();
                      laserCloudCubeSurfPointer->clear();
                  }
              }




              centerCubeI++;
              laserCloudCenWidth++;
          }




          while (centerCubeI >= laserCloudWidth - 3)
          {
              for (int j = 0; j < laserCloudHeight; j++)
              {
                  for (int k = 0; k < laserCloudDepth; k++)
                  {
                      int i = 0;
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      for (; i < laserCloudWidth - 1; i++)
                      {
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      }
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeCornerPointer;
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeSurfPointer;
                      laserCloudCubeCornerPointer->clear();
                      laserCloudCubeSurfPointer->clear();
                  }
              }




              centerCubeI--;
              laserCloudCenWidth--;
          }




          while (centerCubeJ < 3)
          {
              for (int i = 0; i < laserCloudWidth; i++)
              {
                  for (int k = 0; k < laserCloudDepth; k++)
                  {
                      int j = laserCloudHeight - 1;
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      for (; j >= 1; j--)
                      {
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                      }
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeCornerPointer;
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeSurfPointer;
                      laserCloudCubeCornerPointer->clear();
                      laserCloudCubeSurfPointer->clear();
                  }
              }




              centerCubeJ++;
              laserCloudCenHeight++;
          }




          while (centerCubeJ >= laserCloudHeight - 3)
          {
              for (int i = 0; i < laserCloudWidth; i++)
              {
                  for (int k = 0; k < laserCloudDepth; k++)
                  {
                      int j = 0;
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      for (; j < laserCloudHeight - 1; j++)
                      {
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                      }
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeCornerPointer;
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeSurfPointer;
                      laserCloudCubeCornerPointer->clear();
                      laserCloudCubeSurfPointer->clear();
                  }
              }




              centerCubeJ--;
              laserCloudCenHeight--;
          }




          while (centerCubeK < 3)
          {
              for (int i = 0; i < laserCloudWidth; i++)
              {
                  for (int j = 0; j < laserCloudHeight; j++)
                  {
                      int k = laserCloudDepth - 1;
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      for (; k >= 1; k--)
                      {
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                      }
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeCornerPointer;
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeSurfPointer;
                      laserCloudCubeCornerPointer->clear();
                      laserCloudCubeSurfPointer->clear();
                  }
              }




              centerCubeK++;
              laserCloudCenDepth++;
          }




          while (centerCubeK >= laserCloudDepth - 3)
          {
              for (int i = 0; i < laserCloudWidth; i++)
              {
                  for (int j = 0; j < laserCloudHeight; j++)
                  {
                      int k = 0;
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                      for (; k < laserCloudDepth - 1; k++)
                      {
                          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                      }
                      laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeCornerPointer;
                      laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                          laserCloudCubeSurfPointer;
                      laserCloudCubeCornerPointer->clear();
                      laserCloudCubeSurfPointer->clear();
                  }
              }




              centerCubeK--;
              laserCloudCenDepth--;
          }




          int laserCloudValidNum = 0;
          int laserCloudSurroundNum = 0;




          for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
          {
              for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
              {
                  for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
                  {
                      if (i >= 0 && i < laserCloudWidth &&
                          j >= 0 && j < laserCloudHeight &&
                          k >= 0 && k < laserCloudDepth)
                      {
                          laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                          laserCloudValidNum++;
                          laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                          laserCloudSurroundNum++;
                      }
                  }
              }
          }




          laserCloudCornerFromMap->clear();
          laserCloudSurfFromMap->clear();
          for (int i = 0; i < laserCloudValidNum; i++)
          {
              *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
              *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
          }
          int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
          int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();








          pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
          downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
          downSizeFilterCorner.filter(*laserCloudCornerStack);
          int laserCloudCornerStackNum = laserCloudCornerStack->points.size();




          pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
          downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
          downSizeFilterSurf.filter(*laserCloudSurfStack);
          int laserCloudSurfStackNum = laserCloudSurfStack->points.size();




          printf("map prepare time %f ms\n", t_shift.toc());
          printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
          if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
          {
              TicToc t_opt;
              TicToc t_tree;
              kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
              kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
              printf("build tree time %f ms \n", t_tree.toc());




              for (int iterCount = 0; iterCount < 2; iterCount++)
              {
                  //ceres::LossFunction *loss_function = NULL;
                  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                  ceres::Manifold *q_parameterization =
                      new ceres::EigenQuaternionManifold();
                  ceres::Problem::Options problem_options;




                  ceres::Problem problem(problem_options);
                  problem.AddParameterBlock(parameters, 4, q_parameterization);
                  problem.AddParameterBlock(parameters + 4, 3);




                  TicToc t_data;
                  int corner_num = 0;




                  for (int i = 0; i < laserCloudCornerStackNum; i++)
                  {
                      pointOri = laserCloudCornerStack->points[i];
                      //double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
                      pointAssociateToMap(&pointOri, &pointSel);
                      kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);




                      if (pointSearchSqDis[4] < 1.0)
                      {
                          std::vector<Eigen::Vector3d> nearCorners;
                          Eigen::Vector3d center(0, 0, 0);
                          for (int j = 0; j < 5; j++)
                          {
                              Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                                  laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                                  laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                              center = center + tmp;
                              nearCorners.push_back(tmp);
                          }
                          center = center / 5.0;




                          Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                          for (int j = 0; j < 5; j++)
                          {
                              Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                              covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                          }




                          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);




                          // if is indeed line feature
                          // note Eigen library sort eigenvalues in increasing order
                          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                          {
                              Eigen::Vector3d point_on_line = center;
                              Eigen::Vector3d point_a, point_b;
                              point_a = 0.1 * unit_direction + point_on_line;
                              point_b = -0.1 * unit_direction + point_on_line;




                              ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                              problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                              corner_num++; 
                          }                         
                      }
                      /*
                      else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
                      {
                          Eigen::Vector3d center(0, 0, 0);
                          for (int j = 0; j < 5; j++)
                          {
                              Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                                  laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                                  laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                              center = center + tmp;
                          }
                          center = center / 5.0;
                          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                          ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
                          problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                      }
                      */
                  }




                  int surf_num = 0;
                  for (int i = 0; i < laserCloudSurfStackNum; i++)
                  {
                      pointOri = laserCloudSurfStack->points[i];
                      //double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
                      pointAssociateToMap(&pointOri, &pointSel);
                      kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);




                      Eigen::Matrix<double, 5, 3> matA0;
                      Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                      if (pointSearchSqDis[4] < 1.0)
                      {
                        
                          for (int j = 0; j < 5; j++)
                          {
                              matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                              matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                              matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                              //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
                          }
                          // find the norm of plane
                          Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                          double negative_OA_dot_norm = 1 / norm.norm();
                          norm.normalize();




                          // Here n(pa, pb, pc) is unit norm of plane
                          bool planeValid = true;
                          for (int j = 0; j < 5; j++)
                          {
                              // if OX * n > 0.2, then plane is not fit well
                              if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                                       norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                                       norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                              {
                                  planeValid = false;
                                  break;
                              }
                          }
                          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                          if (planeValid)
                          {
                              ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                              problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                              surf_num++;
                          }
                      }
                      /*
                      else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
                      {
                          Eigen::Vector3d center(0, 0, 0);
                          for (int j = 0; j < 5; j++)
                          {
                              Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
                                                  laserCloudSurfFromMap->points[pointSearchInd[j]].y,
                                                  laserCloudSurfFromMap->points[pointSearchInd[j]].z);
                              center = center + tmp;
                          }
                          center = center / 5.0;
                          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                          ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
                          problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                      }
                      */
                  }




                  //printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
                  //printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);




                  printf("mapping data assosiation time %f ms \n", t_data.toc());




                  TicToc t_solver;
                  ceres::Solver::Options options;
                  options.linear_solver_type = ceres::DENSE_QR;
                  options.max_num_iterations = 4;
                  options.minimizer_progress_to_stdout = false;
                  options.check_gradients = false;
                  options.gradient_check_relative_precision = 1e-4;
                  ceres::Solver::Summary summary;
                  ceres::Solve(options, &problem, &summary);
                  printf("mapping solver time %f ms \n", t_solver.toc());




                  //printf("time %f \n", timeLaserOdometry);
                  //printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
                  //printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
                  //     parameters[4], parameters[5], parameters[6]);
              }
              printf("mapping optimization time %f \n", t_opt.toc());
          }
          else
          {
              ROS_WARN("time Map corner and surf num are not enough");
          }
          transformUpdate();




          TicToc t_add;
          for (int i = 0; i < laserCloudCornerStackNum; i++)
          {
              pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);




              int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
              int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
              int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;




              if (pointSel.x + 25.0 < 0)
                  cubeI--;
              if (pointSel.y + 25.0 < 0)
                  cubeJ--;
              if (pointSel.z + 25.0 < 0)
                  cubeK--;




              if (cubeI >= 0 && cubeI < laserCloudWidth &&
                  cubeJ >= 0 && cubeJ < laserCloudHeight &&
                  cubeK >= 0 && cubeK < laserCloudDepth)
              {
                  int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                  laserCloudCornerArray[cubeInd]->push_back(pointSel);
              }
          }




          for (int i = 0; i < laserCloudSurfStackNum; i++)
          {
              pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);




              int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
              int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
              int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;




              if (pointSel.x + 25.0 < 0)
                  cubeI--;
              if (pointSel.y + 25.0 < 0)
                  cubeJ--;
              if (pointSel.z + 25.0 < 0)
                  cubeK--;




              if (cubeI >= 0 && cubeI < laserCloudWidth &&
                  cubeJ >= 0 && cubeJ < laserCloudHeight &&
                  cubeK >= 0 && cubeK < laserCloudDepth)
              {
                  int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                  laserCloudSurfArray[cubeInd]->push_back(pointSel);
              }
          }
          printf("add points time %f ms\n", t_add.toc());




        
          TicToc t_filter;
          for (int i = 0; i < laserCloudValidNum; i++)
          {
              int ind = laserCloudValidInd[i];




              pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
              downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
              downSizeFilterCorner.filter(*tmpCorner);
              laserCloudCornerArray[ind] = tmpCorner;




              pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
              downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
              downSizeFilterSurf.filter(*tmpSurf);
              laserCloudSurfArray[ind] = tmpSurf;
          }
          printf("filter time %f ms \n", t_filter.toc());
        
          TicToc t_pub;
          //publish surround map for every 5 frame
          if (frameCount % 5 == 0)
          {
              laserCloudSurround->clear();
              for (int i = 0; i < laserCloudSurroundNum; i++)
              {
                  int ind = laserCloudSurroundInd[i];
                  *laserCloudSurround += *laserCloudCornerArray[ind];
                  *laserCloudSurround += *laserCloudSurfArray[ind];
              }




              sensor_msgs::PointCloud2 laserCloudSurround3;
              pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
              laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
              laserCloudSurround3.header.frame_id = "/camera_init";
              pubLaserCloudSurround.publish(laserCloudSurround3);
          }




          if (frameCount % 20 == 0)
          {
              pcl::PointCloud<PointType> laserCloudMap;
              for (int i = 0; i < 4851; i++)
              {
                  laserCloudMap += *laserCloudCornerArray[i];
                  laserCloudMap += *laserCloudSurfArray[i];
              }
              sensor_msgs::PointCloud2 laserCloudMsg;
              pcl::toROSMsg(laserCloudMap, laserCloudMsg);
              laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
              laserCloudMsg.header.frame_id = "/camera_init";
              pubLaserCloudMap.publish(laserCloudMsg);
          }

/*직접 추가Start---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        
//point cloud에 color 정보 추가 작업
        
         int xp, yp, B, G, R;
         uchar* p;
         int channels = _mat_left.channels();
      
         pcl::PointXYZRGB pixpoint;

         cv::Mat_<float> homo_2D(3,1); //2D(x,y) of pixel coordinate expressed in homogeneous coordinate(x,y,1)
         cv::Mat_<float> homo_3D(4,1); //3D(x,y,z) of world coordinate expressed in homogeneous coordinate(x,y,z,1)

         _laserCloudFullResColor->clear();

        //초기에 intrinsic parameter값 정보 얻은 후에만 실행
        if(K.at<float>(0,0) != 0 && _newLeftcamInfo == true){ 
           cv::Mat _mat_left_clone = _mat_left.clone();
           cv::Mat _Pose=Pose.clone();
           char temp[256]={'\0'};
  
          //Enter를 누른 시점부터 실시간 Colored Point Cloud를 얻어내기 시작 혹은 중단: 원활한 촬영 방식을 위한 고안법
          //i+Enter를 누른 시점부터 실시간 Image 저장 시작 혹은 중단 
          if(CheckSTDIN(10000)){ //1s=1000000
            fgets(temp, 256, stdin);
            //if temp value == just enter --> getpcd
            if(strcmp(temp,"\n") ==0)
              getPcd*=-1;	
          
            //else if temp value == i+enter --> getImage
            else if(strcmp(temp,"i\n") ==0)
              getImage*=-1;	
          }
      
      		if(getPcd==true){
      		   for (auto& pt : *laserCloudFullRes){
      		       //pose 구하기
      		       if(-3<pt.x && pt.x<3 && pt.y<0) //unit(meter)
      		       {
      			
      		           // 3D point 좌표
            			   homo_3D.at<float>(0,0) = pt.x;
            			   homo_3D.at<float>(1,0) = pt.y;
            			   homo_3D.at<float>(2,0) = pt.z;
            			   homo_3D.at<float>(3,0) = 1;
  
                     //라이다 좌표 -> 카메라 좌표 -> 픽셀 좌표(xp, yp)
      		           homo_2D = KE * homo_3D;  
      		           xp = round(homo_2D.at<float>(0,0)/homo_2D.at<float>(2,0));  // 변환한 x, y 좌표. s를 나눠주어야 함.
      		           yp = round(homo_2D.at<float>(1,0)/homo_2D.at<float>(2,0));
      			   
      		           //범위 확인
      		           if(0 <= xp && xp < img_width)  // 2208*1242 /1280,720 이내의 픽셀 좌표를 가지는 값들에 대해서만 depth값을 추가로 비교.
      		           {
      		               if(0 <= yp && yp < img_height) 
      		               {
                             //투영한 pixel좌표의 R, G, B 값 추출
                             p = _mat_left_clone.ptr<uchar>(yp);
                             B = p[xp*channels + 0];   
                             G = p[xp*channels + 1];
                             R = p[xp*channels + 2];
  
                             //3D point에 좌표, 색상 정보 추가
                             pixpoint.b=B;
              				       pixpoint.g=G;
              				       pixpoint.r=R;
                             pixpoint.x = pt.x; 
                             pixpoint.y = pt.y;
                             pixpoint.z = pt.z;
      				               _laserCloudFullResColor->points.emplace_back(pixpoint);
                         }
      		           }
      		       }
            }
      		}
            
      		//Save an image
      		if(frameCount % 3 == 0 && (getImage==true||getPcd==true) ){
      			std::string filename_image, output_path_img;
      			filename_image=std::to_string(imgcounter) +".jpg";
      			output_path_img="/root/catkin_ws/Dataset/ADOP_dataset/images/";
      			
      			// save current pose & image name(e.g 0.png, 1.png ...)
      			fout_masks << filename_image << std::endl;
      			fout_poses <<_Pose.at<double>(0,0) << " " << _Pose.at<double>(0,1) << " " << _Pose.at<double>(0,2) << " " << _Pose.at<double>(0,3)<< " "<< _Pose.at<double>(0,4) << " " << _Pose.at<double>(0,5) << " " << _Pose.at<double>(0,6) << std::endl;
      			imwrite(output_path_img+filename_image, _mat_left_clone);          
      			std::cout<<"save an image.\n\n\n"<<std::endl;  
      			
            imgcounter++;
      		}
       } 
/*직접 추가End---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        


          int laserCloudFullResNum = laserCloudFullRes->points.size();

          for (int i = 0; i < laserCloudFullResNum; i++)
          {
              pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
		
          }

/*직접 추가Start---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        
	  int laserCloudFullResColorNum = _laserCloudFullResColor->points.size();
          for (int i = 0; i < laserCloudFullResColorNum; i++)
          {
              pointAssociateToMapForColor(&_laserCloudFullResColor->points[i], &_laserCloudFullResColor->points[i]); //local lidar coordinate --> world lidar coordinate
	            mappedPointToCameraWorld(&_laserCloudFullResColor->points[i], &_laserCloudFullResColor->points[i]); //lidar coordinate --> camera coordinate
          }

        //accumulate points
        *_ColorPointAccum += *_laserCloudFullResColor;
      	sensor_msgs::PointCloud2 _ColorPointAccum_msg;
      	pcl::toROSMsg(*_ColorPointAccum, _ColorPointAccum_msg);
      	_ColorPointAccum_msg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        _ColorPointAccum_msg.header.frame_id = "/camera_init";
      	pubColorpointAccum.publish(_ColorPointAccum_msg);
    
	

      	if(int(_ColorPointAccum->points.size())>= LIMIT_POINTSIZE){
      		//save pointcloud partially for no latency
      		std::string filename_pcd, output_path;
      		filename_pcd=std::to_string(filecounter) +".ply";
      		output_path="/root/catkin_ws/Dataset/ply_data/";
      
      		pcl::io::savePLYFileBinary(output_path+filename_pcd, *_ColorPointAccum);
      		std::cout<<"save color point cloud in ply or pcd file format\n\n\n"<<std::endl;  
      
      		filecounter++;
      
      		//clear the _ColorPointAccum
      		_ColorPointAccum->clear();
      	}
         
        sensor_msgs::PointCloud2 laserCloudFullRes3;
        //option1_for color pcd
        //pcl::toROSMsg(*_laserCloudFullResColor, laserCloudFullRes3);
        
        //option2_for origin pcd
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudFullRes3.header.frame_id = "/camera_init";
        
        //velodyne_cloud_reg 토픽으로 pub하는 코드
        pubLaserCloudFullRes.publish(laserCloudFullRes3);
/*직접 추가End---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        
      
           printf("mapping pub time %f ms \n", t_pub.toc());




           printf("whole mapping time %f ms +++++\n", t_whole.toc());




           nav_msgs::Odometry odomAftMapped;
           odomAftMapped.header.frame_id = "/camera_init";
           odomAftMapped.child_frame_id = "/aft_mapped";
           odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
           odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
           odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
           odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
           odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
           odomAftMapped.pose.pose.position.x = t_w_curr.x();
           odomAftMapped.pose.pose.position.y = t_w_curr.y();
           odomAftMapped.pose.pose.position.z = t_w_curr.z();
           pubOdomAftMapped.publish(odomAftMapped);




           geometry_msgs::PoseStamped laserAfterMappedPose;
           laserAfterMappedPose.header = odomAftMapped.header;
           laserAfterMappedPose.pose = odomAftMapped.pose.pose;
           laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
           laserAfterMappedPath.header.frame_id = "/camera_init";
           laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
           pubLaserAfterMappedPath.publish(laserAfterMappedPath);




           static tf::TransformBroadcaster br;
           tf::Transform transform;
           tf::Quaternion q;
           transform.setOrigin(tf::Vector3(t_w_curr(0),
                                           t_w_curr(1),
                                           t_w_curr(2)));
           q.setW(q_w_curr.w());
           q.setX(q_w_curr.x());
           q.setY(q_w_curr.y());
           q.setZ(q_w_curr.z());
           transform.setRotation(q);
           br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));




           frameCount++;
       }
       std::chrono::milliseconds dura(2);
       std::this_thread::sleep_for(dura);
   }
   fout_masks.close();
   fout_poses.close();
}




int main(int argc, char **argv)
{
   ros::init(argc, argv, "laserMapping");
   ros::NodeHandle nh;




   float lineRes = 0;
   float planeRes = 0;
   nh.param<float>("mapping_line_resolution", lineRes, 0.4);
   nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
   printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
   downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);
   downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

/*직접 추가Start---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        
//For ZED
    //ros::Subscriber _subLeftUnRectified  = nh.subscribe("/zed2i/zed_node/rgb_raw/image_raw_color", 1, &imageLeftHandler);
    
    ros::Subscriber _subLeftRectified  = nh.subscribe("/zed2i/zed_node/left/image_rect_color", 1, &imageLeftHandler);
    
    ros::Subscriber _subLeftcamInfo = nh.subscribe<sensor_msgs::CameraInfo>("/zed2i/zed_node/left/camera_info", 1, &leftcamInfoHandler);
    
    //ros::Subscriber _subDepthRectified = nh.subscribe("/zed2i/zed_node/depth/depth_registered", 1, &depthHandler);
    
    //ros::Subscriber _subLeftunrectcamInfo = nh.subscribe<sensor_msgs::CameraInfo>("/zed2i/zed_node/rgb_raw/camera_info", 1, &leftcamInfoHandler);
  // }
/*직접 추가End---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        

   ros::Subscriber _subZedTrans = nh.subscribe<geometry_msgs::PoseStamped>("/zed2i/zed_node/pose", 1, &zedPoseHandler);   // zed2로 부터 월드 좌표계에서의 센서 위치 받아오기.



   ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);




   ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);




   ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);




   ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);







   pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
   //주변 환경을 표현하는 point cloud를 pub하는데 사용함, 이 point cloud는 맵을 업데이트하고 환경에서 새로운 장애물을 인지하는데 사용함




   pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
   //aloam에 의해 처리된 다중 포인트클라우드의 조합으로 축적된 맵 포인트 클라우드임, 최종!!




   pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100); //라이다 센서의 pose를 추정하고 맵을 생성하기 위해 사용되는 point cloud를 담고 있음

/*직접 추가*/        
   pubColorpointAccum = nh.advertise<sensor_msgs::PointCloud2>("/colored_pcd_accum", 100);

   pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);




   pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);




   pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);




   for (int i = 0; i < laserCloudNum; i++)
   {
       laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
       laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
   }
   
   ros_status= ros::ok(); //ros_status = true

/*직접 추가Start---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        
/*	int flag=0;
	do{
		std::cout<<"Do you wanna remove all files? Yes:1 No:0"<<std::endl;
		std::scanf("%d", &flag);
	}while(!(flag==0 || flag==1));
   if(ros_status==true && flag){
      system("rm -rf /root/catkin_ws/Dataset/ply_data/*pcd");
      system("rm -rf /root/catkin_ws/Dataset/ply_data/*ply");
      system("rm -rf /root/catkin_ws/Dataset/ADOP_dataset/images/*jpg");
      std::cout << "All files in the folder have been deleted." << std::endl;
   }*/
	
   if(ros_status==true){
      system("rm -rf /root/catkin_ws/Dataset/ply_data/*pcd");
      system("rm -rf /root/catkin_ws/Dataset/ply_data/*ply");
      system("rm -rf /root/catkin_ws/Dataset/ADOP_dataset/images/*jpg");
      std::cout << "All files in the folder have been deleted." << std::endl;
   }
   

   std::thread mapping_process{process};
  
   ros::spin();
   ros_status= ros::ok(); //ros_status = false
	
   
   /*//If ctrl+C was typed, save the accumulated point cloud and exit
   std::string filename, output_path;
   filename="ColorPcd.ply";
   output_path="/root/catkin_ws/Dataset/ply_data/";
   if(ros_status == false){
	pcl::io::savePLYFileBinary(output_path+filename, *_ColorPointAccum);
	std::cout<<"save color point cloud in ply or pcd file format\n\n\n"<<std::endl;       
   }*/
/*직접 추가End---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/        
  
   return 0;
}

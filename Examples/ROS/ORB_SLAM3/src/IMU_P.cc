/**
* This file is added by chan
*/
#include <iostream>
#include <iomanip>


#include"../../../include/xslam_sdk.hpp"
#include"../../../include/frequency_counter.hpp"

// ROS
#include<algorithm>
#include<fstream>
#include<chrono>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

// CV
#include <image_transport/image_transport.h>
// #ifdef EXAMPLES_WITH_OPENCV
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <memory>
   cv::Mat rgbImage(xslam_rgb *rgb);
// #endif

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h> 

#include"../../../include/System.h"


// created by chan
std_msgs::String imu_msg;
//quoternion, Angular velocity, Linear acceleration
float X_,Y_,Z_,W_,Avx,Avy,Avz,lax,lay,laz;


// camera code created by chan

std::string codec_name( xslam_rgb_codec codec )
{
    switch( codec ){
    case YUYV: return "YUYV";
    case YUV420p: return "YUV420p";
    case JPEG: return "JPEG";
    }
    return "unknown";
}

cv::Mat im;

void show_rgb(xslam_rgb* rgb)
{
  static FrequencyCounter fc;
  static int cnt = 0;

  fc.tic();
  
  if(cnt++ % 5 == 0)
  {
    std::cout
        << "[RGB]: [" << std::setw(8) << rgb->timestamp << std::setw(4) << " s],"
        << " fps=" << static_cast<int>(fc.fps()) << " images_size=("
        << rgb->width << "x" << rgb->height << ") "
        << "codec=" << codec_name(rgb->codec) << " "
        << "data_size=" << rgb->dataSize << std::endl;
        // created by chan

  }

// #ifdef EXAMPLES_WITH_OPENCV
    im = rgbImage( rgb );
    cv::Size s = im.size();
    s.width *= 0.5;
    s.height *= 0.5;
    cv::resize(im, im, s);

    // cv::imshow("RGB", im);
    char c = static_cast<char>(cv::waitKey(1));

// #endif
}
// #ifdef EXAMPLES_WITH_OPENCV
cv::Mat rgbImage(xslam_rgb *rgb)
{
    cv::Mat out;
    switch(rgb->codec){
       case xslam_rgb_codec::YUYV:{
           cv::Mat img( rgb->height, rgb->width, CV_8UC2, rgb->data );
           cv::cvtColor( img, out, cv::COLOR_YUV2RGB_YUYV );
           break;
       }
       case xslam_rgb_codec::YUV420p:{
           cv::Mat img( static_cast<int>(1.5*rgb->height), rgb->width, CV_8UC1, rgb->data );
           cv::cvtColor( img, out, cv::COLOR_YUV420p2RGB );
           break;
       }
       case xslam_rgb_codec::JPEG:{
           cv::Mat img( rgb->height, rgb->width, CV_8UC3, rgb->data );
           out = cv::imdecode( img, cv::IMREAD_COLOR );
           break;
       }
    }

    return out;
}
// #endif

// IMU

void show_frame(xslam_frame*){}
// Display the time and the 6 dof pose in the world coordinate frame
void show_pose(xslam_pose* pose)
{
  static FrequencyCounter fc;
  static int cnt = 0;

  fc.tic();

  if(cnt++ % 300 == 0)
  {
    double fps = fc.fps();
    double distance_to_origin = std::sqrt(pose->x*pose->x + pose->y*pose->y + pose->z*pose->z);
    std::cout
      <<"[6DOF]: [" << std::setw(8) << pose->timestamp << std::setw(4) << " s],"
      << " fps=" << int(fps) << std::setprecision(5)
      << " p=(" << pose->x << " " << pose->y << " " << pose->z
      << " ), r=(" << pose->pitch << " " << pose->yaw << " " << pose->roll << " ), to origin(" << distance_to_origin << ")"
      << ", Confidence= " << (int)pose->confidence << std::endl;
  }
}

// Display the time and the 6 dof pose in the world coordinate frame using quaternion
void show_pose_quaternion(xslam_pose_quaternion* pose)
{
  static FrequencyCounter fc;
  static int cnt = 0;

  fc.tic();

  if(cnt++ % 300 == 0)
  {
    double fps = fc.fps();
    double distance_to_origin = std::sqrt(pose->x[0]*pose->x[0] + pose->x[1]*pose->x[1] + pose->x[2]*pose->x[2]);
    std::cout
      <<"[6DOF]: [" << std::setw(8) << pose->timestamp << std::setw(4) << " s],"
      << " fps=" << int(fps) << std::setprecision(5)
      << " p=(" << pose->x[0] << " " << pose->x[1] << " " << pose->x[2]
      << " ), q=(" << pose->quaternion[0] << " " << pose->quaternion[1] << " " << pose->quaternion[2] << " " << pose->quaternion[3]
      << " ), to origin(" << distance_to_origin << ")"
      << ", Confidence= " << (int)pose->confidence << std::endl;
  }
  // created by chan
  // Quaternion
  X_ = pose->quaternion[0], Y_ = pose->quaternion[1], Z_ = pose->quaternion[2], W_ = pose->quaternion[3];

}

// Function to call for each IMU info
void show_imu(xslam_imu* imu)
{
  static FrequencyCounter fc;
  static int cnt = 0;

  fc.tic();


  if(cnt++ % 100 == 0)
  {
    std::cout
      << "[IMU] : [" << std::setw(8) << imu->timestamp << std::setw(4) << " s],"
      << " fps=" << int(fc.fps())
      << " Gyro=(" << imu->gyro[0] << " " << imu->gyro[1] << " " << imu->gyro[2] << "),"
      << " Accel=("  << imu->accel[0] << " " << imu->accel[1] << " " << imu->accel[2] << "),"
      << " Magn=(" << imu->magn[0] << " " << imu->magn[1] << " " << imu->magn[2] << ")";
    if( imu->temperature > 0 ){
      std::cout << ", Temp=" << (imu->temperature - 273.15f);
    }
    std::cout << std::endl;
  }

  // created by chan
  lax = imu->accel[0], lay = imu->accel[1], laz = imu->accel[2];
  Avx = imu->gyro[0], Avy = imu->gyro[1], Avz = imu->gyro[1];
}

void lost(float timestamp)
{
  std::cout << "[LOST] Device is lost at timestamp " << timestamp << " sec." << std::endl;
}

int main(int argc, char **argv)
{
  // created by chan
  ros::init(argc, argv, "imu_topic");
  ros::NodeHandle nh;
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu",30);
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 30);
  sensor_msgs::Imu imu_data;
  
  //
  // xslam_disp_version();

  // set the function to call for each 6 dof pose, the protopy must be "(void)(xslam_pose*)"
  xslam_6dof_callback(&show_pose);

  // set the function to call for each 6 dof pose with quaternion format
  xslam_6dof_quaternion_callback(&show_pose_quaternion);

  // set the IMU callback
  xslam_imu_callback(&show_imu);

  // set the lost callback
  xslam_lost_callback(&lost);

  // rgb_camera
  xslam_rgb_callback(&show_rgb);
  xslam_start_camera();

  // start visual odometry
  // xslam_start_vo();

  // created by chan
  ros::Rate loop_rate(15);
  while(nh.ok())
  {
      imu_data.header.stamp = ros::Time::now();
      // imu_data.header.frame_id = "map";
      imu_data.orientation.x = X_;
      imu_data.orientation.y = Y_;
      imu_data.orientation.z = Z_;
      imu_data.orientation.w = W_;
      imu_data.angular_velocity.x = Avx;
      imu_data.angular_velocity.y = Avy;
      imu_data.angular_velocity.z = Avz;
      imu_data.linear_acceleration.x = lax;
      imu_data.linear_acceleration.y = lay;
      imu_data.linear_acceleration.z = laz;

      // publish
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im).toImageMsg();
      imu_pub.publish(imu_data);
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
  }

  std::cout << " Press Enter to stop the process" << std::endl;
  std::cin.get();
  std::cout << " Stop process ..." << std::endl;

  // stop visual odometry
  xslam_stop();

  // free ressources
  return xslam_free() == xslam_status::failure ? EXIT_FAILURE : EXIT_SUCCESS;
}


/**
* This file is added by chan
*/
#include <iostream>
#include <iomanip>


#include"../../../include/xslam_sdk.hpp"
#include"../../../include/frequency_counter.hpp"
#include <image_transport/image_transport.h>
// #ifdef EXAMPLES_WITH_OPENCV
    #include <opencv2/highgui/highgui.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <memory>
   cv::Mat rgbImage(xslam_rgb *rgb);
// #endif

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

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h> 

#include"../../../include/System.h"

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


int main(int argc, char **argv)
{
  // created by chan
  ros::init(argc, argv, "camera_topic");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 30);
  

    //   
//   xslam_disp_version();

  xslam_rgb_callback(&show_rgb);

//   chan

  std::cout << " Start process ..." << std::endl;
  // created by chan
  xslam_start_camera();
    // 
    ros::Rate loop_rate(15);
        while(nh.ok())
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im).toImageMsg();
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
  std::cout << " Press \'q\' then Enter to stop the process" << std::endl;

  std::cout << " Stop process ..." << std::endl;

  // stop slam
  xslam_stop();

  // free ressources
  return xslam_free() == xslam_status::failure ? EXIT_FAILURE : EXIT_SUCCESS;
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

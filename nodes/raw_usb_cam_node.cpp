/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <stdio.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CompressedImage.h>
#include <usb_cam/usb_cam.h>

class RawUsbCamNode
{
public:
  ros::NodeHandle node_;
  sensor_msgs::Image img_;

  std::string video_device_name_;
  std::string io_method_name_;
  int image_width_,image_height_, framerate_;
  std::string pixel_format_name_;
  bool autofocus_;
  usb_cam_io_method io_method_;

  std::string camera_name_;

  ros::Time next_time_;
  int count_;

  usb_cam_camera_image_t* camera_image_;

  ros::Publisher image_pub_;

  RawUsbCamNode() :
    node_("~")
  {
    image_pub_ = node_.advertise<sensor_msgs::CompressedImage>("image_raw/compressed", 1);

    node_.param("video_device", video_device_name_, std::string("/dev/video6"));
    node_.param("io_method", io_method_name_, std::string("mmap")); // possible values: mmap, read, userptr
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    node_.param("autofocus", autofocus_, false); // enable/disable autofocus


    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));

    ROS_INFO("Camera name: %s", camera_name_.c_str());
    ROS_INFO("usb_cam video_device set to [%s]\n", video_device_name_.c_str());
    ROS_INFO("usb_cam io_method set to [%s]\n", io_method_name_.c_str());
    ROS_INFO("usb_cam image_width set to [%d]\n", image_width_);
    ROS_INFO("usb_cam image_height set to [%d]\n", image_height_);
    ROS_INFO("usb_cam auto_focus set to [%d]\n", autofocus_);

    if(io_method_name_ == "mmap")
      io_method_ = IO_METHOD_MMAP;
    else if(io_method_name_ == "read")
      io_method_ = IO_METHOD_READ;
    else if(io_method_name_ == "userptr")
      io_method_ = IO_METHOD_USERPTR;
    else {
      ROS_FATAL("Unknown io method.");
      node_.shutdown();
      return;
    }

    camera_image_ = NULL;
    while (!ros::isShuttingDown() && camera_image_ == NULL) {
        camera_image_ = usb_cam_camera_start(video_device_name_.c_str(),
                                         io_method_,
                                         PIXEL_FORMAT_MJPEG,
                                         image_width_,
                                         image_height_,
                                         framerate_);
      if (camera_image_ == NULL) {
          usleep(1000000);
          // Fetch the video device again, in case it changed
          node_.param("video_device", video_device_name_, std::string("/dev/video6"));
      }
    }

    if(autofocus_) {
      usb_cam_camera_set_auto_focus(1);
    }

    next_time_ = ros::Time::now();
    count_ = 0;
  }

  virtual ~RawUsbCamNode()
  {

    usb_cam_camera_shutdown();
  }

  bool take_and_send_image()
  {
    sensor_msgs::CompressedImage image;
    image.format = "jpeg";

    usb_cam_camera_grab_mjpeg(&image.data);
    image.header.stamp = ros::Time::now();

    image_pub_.publish(image);
    return true;
  }

  bool spin()
  {
    while (node_.ok())
    {
      if (take_and_send_image())
      {
        count_++;
        ros::Time now_time = ros::Time::now();
        if (now_time > next_time_) {
          ROS_DEBUG("%d frames/sec", count_);
          count_ = 0;
          next_time_ = next_time_ + ros::Duration(1,0);
        }
      } else {
        ROS_ERROR("couldn't take image.");
        usleep(1000000);
      }
      std::string device_name;
      node_.getParam("video_device", device_name);
      if (device_name != video_device_name_) {
          usb_cam_camera_shutdown();
          camera_image_ = usb_cam_camera_start(device_name.c_str(),
                                         io_method_,
                                         PIXEL_FORMAT_MJPEG,
                                         image_width_,
                                         image_height_,
                                         framerate_);
          if (camera_image_ == NULL) {
              usleep(1000000);
              camera_image_ = usb_cam_camera_start(video_device_name_.c_str(),
                                             io_method_,
                                             PIXEL_FORMAT_MJPEG,
                                             image_width_,
                                             image_height_,
                                             framerate_);
          }
          else {
              video_device_name_ = device_name;
          }
      }
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "raw_usb_cam");
  for (int i = 0; i < argc; i++) {
      std::cout << argv[i] << std::endl;
  }
  RawUsbCamNode a;
  a.spin();
  return 0;
}

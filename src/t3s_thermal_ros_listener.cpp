#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <math.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/core/core_c.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <iostream>
// #include "t3s_thermal_ros_wrapper/SimplePictureProcessing.h"
// #include "t3s_thermal_ros_wrapper/thermometry.h"
//#include "thermometry.h"
//#include "SimplePictureProcessing.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "t3s_thermal_ros_wrapper/t3s_thermal.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

using namespace std;

void thermal_msg_Callback(const t3s_thermal_ros_wrapper::t3s_thermalConstPtr &msg)
{
    // std::vector<float, std::allocator<float>>* temperaturedata_;
    // *temperaturedata_ = msg->temperaturedata;
    ROS_INFO("Thermal Mat Info:\nframe_seq%d\ntimestamp%lld\nMatrixSize%d\ncenterTmp:%.2f,maxTmp:%.2f,minTmp:%.2f,avgTmp:%.2f",
            msg->header.seq,
            msg->timestamp_t3s_sec*1000000 + msg->timestamp_t3s_usec,
            msg->temperaturedata.size(),
            msg->temperaturedata[0],
            msg->temperaturedata[3],
            msg->temperaturedata[6],
            msg->temperaturedata[9]
            );
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "t3s_thermal_msg_listener_test");
    ros::NodeHandle nh;
    ros::Subscriber t3s_thermal_msg_sub = nh.subscribe("/t3s_camera/t3s_thermal_msg",1, thermal_msg_Callback);


    ros ::spin();
    return 0;

}


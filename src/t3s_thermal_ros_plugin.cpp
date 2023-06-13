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
#include "t3s_thermal_ros_wrapper/SimplePictureProcessing.h"
#include "t3s_thermal_ros_wrapper/thermometry.h"
//#include "thermometry.h"
//#include "SimplePictureProcessing.h"

#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include "std_msgs/String.h"
#include "t3s_thermal_ros_wrapper/t3s_thermal.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/Image.h>
#include "image_transport/image_transport.h"


using namespace std;
using namespace cv;

/**
 * OUTPUTMODE 4:配合libthermometry.so可以输出全局温度数据。
 *              配合simple.so使用专业级图像算法，可得到优秀画质的图像，但是需要主频1.8ghz。
 *              也可配合代码里的线性图像算法，画质稍低于高性能算法，但是对主频几乎没要求。
 *              输出数据格式：按照yuyv的形式，实际输出每个像素16bit中14bit为灰度数据，最后四行为参数。
 * OUTPUTMODE 4:Combine with libthermometry.so to output all around temperature data.
 *              Obtain high quality image with professional algorithms from simple.so, requires basic frequency at least 1.8ghz.
 *              Linear Algorithms produce lower quality image than professional algorithms but there are almost no requirements in basic frequency.
 *              Output data format: in yuyu form, the actual transferred is 16bit NUC data with 14 bits to store the wide dynamic grayscale value of one pixel.
 *              The last four lines of yuyu data are parameters.
 *   
 * 
 * OUTPUTMODE 5:配合libthermometry.so，可以直接输出中心点，最高温，最低温和额外指定的三个点的信息，不可输出全帧温度数据。              
 *              输出数据格式：输出yuyv格式的图像，最后四行为参数。
 *  OUTPUTMODE 5:With libthermometry.so to directly output temperature of the center, highest, lowest and extra three points. Can't output full frame temperature data.
 *               Output data format: graphs in yuyv format, the last four lines are parameters. 
 */
#define OUTPUTMODE 4
//#define OUTPUTMODE 5

#define TRUE 1
#define FALSE 0
#define MAX_BUFFER 2048

#define FILE_VIDEO1 "video"
#define FILE_DEV "/dev"


namespace t3s_thermal_ns
{
    class T3s_Thermal_Nodelet_Class : public nodelet::Nodelet
    {
        public:
            T3s_Thermal_Nodelet_Class() {;}
            ~T3s_Thermal_Nodelet_Class() {;}
        private:
            //注意配置和设备相应的分辨率 Caution: set resolution that correspond to specification of the device
            //#define IMAGEWIDTH 384
            //#define IMAGEHEIGHT 292
            //#define IMAGEWIDTH 640
            //#define IMAGEHEIGHT 516
            int IMAGEWIDTH = 384;
            int IMAGEHEIGHT = 292;

            static int fd;                          //设备描述符 Descriptor of device 
            struct v4l2_streamparm stream_para;     //结构体v4l2_streamparm来描述视频流的属性 Struct v4l2_streamparm to describe the property of video stream
            struct v4l2_capability cap;             //取得设备的capability，看看设备具有什么功能，比如是否具有视频输入,或者音频输入输出等 
                                                    //Obtain capability of device, check if it has video input or audio input/output abilities.
            struct v4l2_fmtdesc fmtdesc;            //枚举设备所支持的image format:  VIDIOC_ENUM_FMT    Enumerate image format the deivce supported: VIDIOC_ENUM_FMT
            struct v4l2_format fmt,fmtack;          //子结构体struct v4l2_pix_format设置摄像头采集视频的宽高和类型：V4L2_PIX_FMT_YYUV V4L2_PIX_FMT_YUYV  
                                                    //Substruct struct v4l2_pix_format for setting Height, Width and Type of captured video 
            struct v4l2_requestbuffers req;         //向驱动申请帧缓冲的请求，里面包含申请的个数 Send request of frame buffer from driver, including amount of requests
            struct v4l2_buffer buf;                 //代表驱动中的一帧 One frame in the driver 

            struct buffer//从相机获得的数据缓存 Capture data buffer from Camera
            {
                void * start;
                unsigned int length;
                long long int timestamp;
            } *buffers;

            struct irBuffer//使用专业级图像算法所需要的缓存   The buffer required by professional algorithm
            {
                size_t** midVar;
                unsigned char* destBuffer;
            } *irBuffers;

            /**
             *temperatureData:最终输出的温度数据，采用10+全帧温度数据格式；例如10+384（宽）×288（高），前10个格式如下 
            *     The final output temperature data, in the form of "10 + Full Frame Temperature Data"; such as 10+384(width)×288(height), the top 10 as below
            *temperatureData[0]=centerTmp;
            *temperatureData[1]=(float)maxx1;
            *temperatureData[2]=(float)maxy1;
            *temperatureData[3]=maxTmp;
            *temperatureData[4]=(float)minx1;
            *temperatureData[5]=(float)miny1;
            *temperatureData[6]=minTmp;
            *temperatureData[7]=point1Tmp;
            *temperatureData[8]=point2Tmp;
            *temperatureData[9]=point3Tmp;
            *根据8004或者8005模式来查表，8005模式下仅输出以上注释的10个参数，8004模式下数据以上参数+全局温度数据
            *比如（x，y）点，第x行y列，位置在temperatureData[10+width*x+y],原点为（0，0）
            *Search on the table with 8004 or 8005 mode, 8005 mode only outputs the 10 parameters above, 8004 mode include above parameters with overall temperature data
            *For example, point (x, y), row x, column y, location in temperatureData[10+width*x+y], origin is (0,0)
            *参见：thermometrySearch函数 Refer to function thermometrySearch
            */
            float* temperatureData;
            /**
             *设置三个单独点 Set three points
            *温度会出现在temperatureData[7]，temperatureData[8]，temperatureData[9] shows temperature
            *0<=viewX1<IMAGEWIDTH
            *0<=viewY1<IMAGEHEIGHT-4
            */
            void setPoint(int viewX1,int viewY1,int indexOfPoint);
            enum   v4l2_buf_type type;//帧类型  Type of buffer
            struct v4l2_control ctrl;

            /**
             *temperatureTable:温度映射表 14bit->16384
            */
            float temperatureTable[16384];

            int init_v4l2(string videoX);           //初始化 Initialization
            int v4l2_grab(void);                    //采集 Capture
            int v4l2_control(int);                  //控制 Control
            int traversalVideo(void);               //遍历video，如果是多个UVC设备可以在此处增加判断，是不是红外UVC  
                                                    //Traveral video, may add determine statements if there are multiple UVC devices, weither infrared UVC

            int delayy;
            void sendCorrection(float correction);   //设置修正，一般取（-3.0)-3.0,用于整体温度向上或者向下修正  Set correction, usally -3.0/3.0 to correct temperature lower or higher

            void sendReflection(float reflection);   //设置反射温度，一般情况下为环境温度  Set reflection temperature, usually ambient temperature
            /*反射温度的确定：
            当周围没有热源时，目标反射的是环境温度，所以一般情况下反射温度等于环境温度。
            当周围有热源时，目标会反射热源的温度。如何确定反射温度：
            1）取一张铝箔（红外反射镜），弄皱后再展平（朗伯面），将铝箔放在纸板上，亮面朝上。
            2）调节热像仪发射率为1。
            3）将铝箔放在目标表面前面并与之平行，用热像仪测量反射镜表面温度，此温度即反射温度。*/
            /*How to find out reflection temperature:
            When there is no heat source nearby, the object reflects ambient temperature, so usually refection equals ambient temperature.
            When there is heat source, object reflects temperature of heat source. How to know the reflection temperature:
            1)Take an aluminum foil as Mirror to reflect infrared ray, wrinkle the foil and then flat it. Put the foil on cardboard, the bright surface upwards  
            2)Adjust the emissivity of device to 1;
            3)Parallel foil with object, measure surface temperature of foil with thermal imager. The measured temperature is reflection temperature.
            */


            void sendAmb(float amb);                        //设置环境温度   Set ambient temperature

            void sendHumidity(float humidity);              //设置湿度（0-1.0），一般0.45   Set humidity (0-1.0), usually 0.45 

            void sendEmissivity(float emiss);               //发射率（0-1.0），具体发射率见附表数据   Emissivity(0-1.0), refer to emissivity table

            void sendDistance(unsigned short distance);              //距离（单位米）  Distance (Unit: Meter)   
            void savePara();                                //保存参数到机芯，断电不丢失   Save parameter to the chipset
            int v4l2_release();                             //释放v4l2  Release v4l2




            cv::Mat rgbImg;
            cv::Mat orgLumiUmgL;
            cv::Mat orgRgbImgP;
            ros::Publisher t3s_thermal_temp_inf_pub;
            image_transport::Publisher t3s_thermal_gray_img_pub;
            image_transport::Publisher t3s_thermal_rgb_img_pub;


            //测温相关参数，详见thermometry.h    Refer to thermometry.h for relevant parameters
            int rangeMode=120;
            float floatFpaTmp;
            float correction;
            float Refltmp;
            float Airtmp;
            float humi;
            float emiss;
            unsigned short distance;
            //以下cameraLens为68：1、输出为384×292，镜头=<6.8mm 2、输出为256×196，镜头=<4mm.   其他默认130
            //The following cameraLens is 68: 1,the output is 384×292, lens =<6.8mm 2, the output is 256×196, lens =<4mm.  Other default 130
            int cameraLens=130;
            float shutterFix=0;
            //end -测温相关参数

            char sn[32];//camera序列码   camera serial number
            char cameraSoftVersion[16];//camera软件版本   camera Software Version
            unsigned short shutTemper;
            float floatShutTemper;//快门温度    Shutter Temperature
            unsigned short coreTemper;
            float floatCoreTemper;//外壳温度   Shell temperature
            const unsigned char* paletteIronRainbow = getPalette(0);//256*3 铁虹   Iron Rainbow
            const unsigned char* palette3 = getPalette(1);//256*3 彩虹1    Rainbow 1
            const unsigned char* paletteRainbow = getPalette(2);//224*3 彩虹2    Rainbow 2
            const unsigned char* paletteHighRainbow = getPalette(3);//448*3 高动态彩虹   HDR rainbow   
            const unsigned char* paletteHighContrast = getPalette(4);//448*3 高对比彩虹    High Contrast rainbow
            
            double t;
            long long int extra_time = 0;
            long long int cur_time = 0;
            long long int last_time = 0;
            







            virtual void onInit()
            {
                ros::NodeHandle& nh = getMTPrivateNodeHandle();
                image_transport::ImageTransport it(nh);
                t3s_thermal_gray_img_pub = it.advertise("/t3s_thermal_img_gray",1);
                t3s_thermal_rgb_img_pub = it.advertise("/t3s_thermal_img_rgb",1);
                


                //T3s_init
                printf("first~~\n");
                if(traversalVideo() == FALSE)       //打开摄像头
                {
                    printf("Init fail~~\n");
                    exit(EXIT_FAILURE);
                }

                //初始化专业级图像算法 Initialize Professional Algorithm
                irBuffers = (irBuffer*)malloc(4 * sizeof(*irBuffers));
                if(!irBuffers)
                {
                    printf("Out of memory\n");
                    return 0;
                }
                if(OUTPUTMODE==4)
                {
                    SimplePictureProcessingInit(IMAGEWIDTH,(IMAGEHEIGHT-4));
                    SetParameter(100,0.5f,0.1f,0.1f,1.0f,3.5f);
                }
                unsigned int n_buffers;
                for(n_buffers = 0; n_buffers < 4; n_buffers++)
                {
                    if(OUTPUTMODE==4)
                    {
                        irBuffers[n_buffers].midVar=(size_t**)calloc (7,sizeof(size_t*));
                        SimplePictureProcessingInitMidVar(irBuffers[n_buffers].midVar);
                    }
                    irBuffers[n_buffers].destBuffer=(unsigned char*)calloc(IMAGEWIDTH*(IMAGEHEIGHT-4)*4,sizeof(unsigned char));
                }
                //end -初始化高性能图像算法

                temperatureData=(float*)calloc(IMAGEWIDTH*(IMAGEHEIGHT-4)+10,sizeof(float));


                printf("second~~\n");
                if(v4l2_grab() == FALSE)
                {
                    printf("grab fail~~\n");
                    exit(EXIT_FAILURE);
                }

                printf("fourth~~\n");
                if(OUTPUTMODE==4)
                {
                    if(v4l2_control(0x8004) == FALSE)//控制机芯切换为8004 原始数据输出   Switch to 8004 mode, output raw data 
                    {
                        printf("control fail~~\n");
                        exit(EXIT_FAILURE);
                    }
                }
                else
                {
                    if(v4l2_control(0x8005) == FALSE)//控制机芯切换为8005 yuyv输出    Switch to 8005 mode, output yuyv
                    {
                        printf("control fail~~\n");
                        exit(EXIT_FAILURE);
                    }
                }

                delayy=0;
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;           //Stream 或者Buffer的类型。此处肯定为V4L2_BUF_TYPE_VIDEO_CAPTURE  Stream or Buffer Type, constantly as V4L2_BUF_TYPE_VIDEO_CAPTURE
                buf.memory = V4L2_MEMORY_MMAP;                    //Memory Mapping模式，则此处设置为：V4L2_MEMORY_MMAP   Memory Mapping mode，set as V4L2_MEMORY_MMAP
                printf("third~~\n");



            }
    };
}
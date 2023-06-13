

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <math.h>
#include <opencv4/opencv2/core/core_c.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <iostream>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include "t3s_thermal_ros_wrapper/SimplePictureProcessing.h"
#include "t3s_thermal_ros_wrapper/thermometry.h"
//#include "thermometry.h"
//#include "SimplePictureProcessing.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "t3s_thermal_ros_wrapper/t3s_thermal.h"

using namespace std;
using namespace cv;

/**
 * OUTPUTMODE 4:配合libthermometry.so可以输出全局温度数据。
 *              配合simple.so使用专业级图像算法，可得到优秀画质的图像，但是需要主频1.8ghz。
 *              也可配合代码里的线性图像算法，画质稍低于高性能算法，但是对主频几乎没要求。
 *              输出数据格式：按照yuyv的形式，实际输出每个像素16bit中14bit为灰度数据，最后四行为参数。
 * OUTPUTMODE 4:Combine with libthermometry.so to output all around temperature
 * data. Obtain high quality image with professional algorithms from simple.so,
 * requires basic frequency at least 1.8ghz. Linear Algorithms produce lower
 * quality image than professional algorithms but there are almost no
 * requirements in basic frequency. Output data format: in yuyu form, the actual
 * transferred is 16bit NUC data with 14 bits to store the wide dynamic
 * grayscale value of one pixel. The last four lines of yuyu data are
 * parameters.
 *
 *
 * OUTPUTMODE
 * 5:配合libthermometry.so，可以直接输出中心点，最高温，最低温和额外指定的三个点的信息，不可输出全帧温度数据。
 *              输出数据格式：输出yuyv格式的图像，最后四行为参数。
 *  OUTPUTMODE 5:With libthermometry.so to directly output temperature of the
 * center, highest, lowest and extra three points. Can't output full frame
 * temperature data. Output data format: graphs in yuyv format, the last four
 * lines are parameters.
 */
#define OUTPUTMODE 4
//#define OUTPUTMODE 5

#define TRUE 1
#define FALSE 0
#define MAX_BUFFER 2048

#define FILE_VIDEO1 "video"
#define FILE_DEV "/dev"
//注意配置和设备相应的分辨率 Caution: set resolution that correspond to
// specification of the device #define IMAGEWIDTH 384 #define IMAGEHEIGHT 292
//#define IMAGEWIDTH 640
//#define IMAGEHEIGHT 516
int IMAGEWIDTH = 384;
int IMAGEHEIGHT = 292;

static int fd;  //设备描述符 Descriptor of device
struct v4l2_streamparm
    stream_para;  //结构体v4l2_streamparm来描述视频流的属性 Struct
                  // v4l2_streamparm to describe the property of video stream
struct v4l2_capability
    cap;  //取得设备的capability，看看设备具有什么功能，比如是否具有视频输入,或者音频输入输出等
          // Obtain capability of device, check if it has video input or audio
          // input/output abilities.
struct v4l2_fmtdesc
    fmtdesc;  //枚举设备所支持的image format:  VIDIOC_ENUM_FMT    Enumerate
              // image format the deivce supported: VIDIOC_ENUM_FMT
struct v4l2_format fmt,
    fmtack;  //子结构体struct
             // v4l2_pix_format设置摄像头采集视频的宽高和类型：V4L2_PIX_FMT_YYUV
             // V4L2_PIX_FMT_YUYV Substruct struct v4l2_pix_format for setting
             // Height, Width and Type of captured video
struct v4l2_requestbuffers
    req;  //向驱动申请帧缓冲的请求，里面包含申请的个数 Send request of frame
          // buffer from driver, including amount of requests
struct v4l2_buffer buf;  //代表驱动中的一帧 One frame in the driver

struct buffer  //从相机获得的数据缓存 Capture data buffer from Camera
{
  void* start;
  unsigned int length;
  long long int timestamp;
} * buffers;

struct irBuffer  //使用专业级图像算法所需要的缓存   The buffer required by
                 // professional algorithm
{
  size_t** midVar;
  unsigned char* destBuffer;
} * irBuffers;

/**
 *temperatureData:最终输出的温度数据，采用10+全帧温度数据格式；例如10+384（宽）×288（高），前10个格式如下
 *     The final output temperature data, in the form of "10 + Full Frame
 *Temperature Data"; such as 10+384(width)×288(height), the top 10 as below
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
 *Search on the table with 8004 or 8005 mode, 8005 mode only outputs the 10
 *parameters above, 8004 mode include above parameters with overall temperature
 *data For example, point (x, y), row x, column y, location in
 *temperatureData[10+width*x+y], origin is (0,0) 参见：thermometrySearch函数
 *Refer to function thermometrySearch
 */
float* temperatureData;
/**
 *设置三个单独点 Set three points
 *温度会出现在temperatureData[7]，temperatureData[8]，temperatureData[9] shows
 *temperature 0<=viewX1<IMAGEWIDTH 0<=viewY1<IMAGEHEIGHT-4
 */
void setPoint(int viewX1, int viewY1, int indexOfPoint);
enum v4l2_buf_type type;  //帧类型  Type of buffer
struct v4l2_control ctrl;

/**
 *temperatureTable:温度映射表 14bit->16384
 */
float temperatureTable[16384];

int init_v4l2(string videoX);  //初始化 Initialization
int v4l2_grab(void);           //采集 Capture
int v4l2_control(int);         //控制 Control
int traversalVideo(
    void);  //遍历video，如果是多个UVC设备可以在此处增加判断，是不是红外UVC
            // Traveral video, may add determine statements if there are
            // multiple UVC devices, weither infrared UVC

int delayy;
void sendCorrection(
    float
        correction);  //设置修正，一般取（-3.0)-3.0,用于整体温度向上或者向下修正
                      // Set correction, usally -3.0/3.0 to
                      // correct temperature lower or higher

void sendReflection(
    float reflection);  //设置反射温度，一般情况下为环境温度  Set reflection
                        // temperature, usually ambient temperature
/*反射温度的确定：
当周围没有热源时，目标反射的是环境温度，所以一般情况下反射温度等于环境温度。
当周围有热源时，目标会反射热源的温度。如何确定反射温度：
1）取一张铝箔（红外反射镜），弄皱后再展平（朗伯面），将铝箔放在纸板上，亮面朝上。
2）调节热像仪发射率为1。
3）将铝箔放在目标表面前面并与之平行，用热像仪测量反射镜表面温度，此温度即反射温度。*/
/*How to find out reflection temperature:
When there is no heat source nearby, the object reflects ambient temperature, so
usually refection equals ambient temperature. When there is heat source, object
reflects temperature of heat source. How to know the reflection temperature:
1)Take an aluminum foil as Mirror to reflect infrared ray, wrinkle the foil and
then flat it. Put the foil on cardboard, the bright surface upwards 2)Adjust the
emissivity of device to 1; 3)Parallel foil with object, measure surface
temperature of foil with thermal imager. The measured temperature is reflection
temperature.
*/

void sendAmb(float amb);  //设置环境温度   Set ambient temperature

void sendHumidity(float humidity);  //设置湿度（0-1.0），一般0.45   Set humidity
                                    //(0-1.0), usually 0.45

void sendEmissivity(
    float emiss);  //发射率（0-1.0），具体发射率见附表数据   Emissivity(0-1.0),
                   // refer to emissivity table

void sendDistance(
    unsigned short distance);  //距离（单位米）  Distance (Unit: Meter)
void savePara();  //保存参数到机芯，断电不丢失   Save parameter to the chipset
int v4l2_release();  //释放v4l2  Release v4l2

int main(int argc, char** argv) {
  // ros wrapper init
  ros::init(argc, argv, "t3s_thermal_ros_wrapper");
  ros::NodeHandle h;
//   image_transport::ImageTransport ywy_thermal_img_trans(h);
  t3s_thermal_ros_wrapper::t3s_thermal t3s_thermal_msg;
  ros::Publisher t3s_thermal_inf_pub =
      h.advertise<t3s_thermal_ros_wrapper::t3s_thermal>(
          "/t3s_camera/t3s_thermal_msg", 1);
//   image_transport::Publisher ywy_thermal_img_pub =
//       ywy_thermal_img_trans.advertise("/thermal_camera/image", 1);
  ros::Publisher pub_img = h.advertise<sensor_msgs::Image>("/thermal_camera/image", 1);
  ros::Rate looprate(30);

  printf("first~~\n");
  if (traversalVideo() == FALSE)  //打开摄像头
  {
    printf("Init fail~~\n");
    exit(EXIT_FAILURE);
  }

  //初始化专业级图像算法 Initialize Professional Algorithm
  irBuffers = (irBuffer*)malloc(4 * sizeof(*irBuffers));
  if (!irBuffers) {
    printf("Out of memory\n");
    return 0;
  }
  if (OUTPUTMODE == 4) {
    SimplePictureProcessingInit(IMAGEWIDTH, (IMAGEHEIGHT - 4));
    SetParameter(100, 0.5f, 0.1f, 0.1f, 1.0f, 3.5f);
  }
  unsigned int n_buffers;
  for (n_buffers = 0; n_buffers < 4; n_buffers++) {
    if (OUTPUTMODE == 4) {
      irBuffers[n_buffers].midVar = (size_t**)calloc(7, sizeof(size_t*));
      SimplePictureProcessingInitMidVar(irBuffers[n_buffers].midVar);
    }
    irBuffers[n_buffers].destBuffer = (unsigned char*)calloc(
        IMAGEWIDTH * (IMAGEHEIGHT - 4) * 4, sizeof(unsigned char));
  }
  // end -初始化高性能图像算法

  temperatureData =
      (float*)calloc(IMAGEWIDTH * (IMAGEHEIGHT - 4) + 10, sizeof(float));

  printf("second~~\n");
  if (v4l2_grab() == FALSE) {
    printf("grab fail~~\n");
    exit(EXIT_FAILURE);
  }

  printf("fourth~~\n");
  if (OUTPUTMODE == 4) {
    if (v4l2_control(0x8004) == FALSE)  //控制机芯切换为8004 原始数据输出 Switch
                                        // to 8004 mode, output raw data
    {
      printf("control fail~~\n");
      exit(EXIT_FAILURE);
    }
  } else {
    if (v4l2_control(0x8005) == FALSE)  //控制机芯切换为8005 yuyv输出    Switch
                                        // to 8005 mode, output yuyv
    {
      printf("control fail~~\n");
      exit(EXIT_FAILURE);
    }
  }

  delayy = 0;
  buf.type =
      V4L2_BUF_TYPE_VIDEO_CAPTURE;  // Stream
                                    // 或者Buffer的类型。此处肯定为V4L2_BUF_TYPE_VIDEO_CAPTURE
                                    // Stream or Buffer Type, constantly
                                    // as V4L2_BUF_TYPE_VIDEO_CAPTURE
  buf.memory =
      V4L2_MEMORY_MMAP;  // Memory Mapping模式，则此处设置为：V4L2_MEMORY_MMAP
                         // Memory Mapping mode，set as V4L2_MEMORY_MMAP
  printf("third~~\n");
  int i = 100;
  double t;
  long long int extra_time = 0;
  long long int cur_time = 0;
  long long int last_time = 0;
  cv::Mat rgbImg(
      IMAGEHEIGHT - 4, IMAGEWIDTH,
      CV_8UC4);  // 8004模式：专业级算法生成的rgba图像，cv使用bgra格式，所以除了黑热和白热，其他伪彩存在通道偏差。8005模式：yuyv直接转化为bgra图像。
                 // 8004 mode: professional algorithm formed rgba image, CV with
                 // bgra format. So there is channel deviation in pseudo color,
                 // except Black Hot and White Hot. 8005 mode: convert yuyv into
                 // bgra image
  cv::Mat orgLumiImgL(
      IMAGEHEIGHT - 4, IMAGEWIDTH,
      CV_8UC1);  // 8004模式使用线性算法生成的灰度图像。In 8004 mode, generate
                 // greyscale via linear algorithm.
  cv::Mat orgRgbImgP(
      IMAGEHEIGHT - 4, IMAGEWIDTH,
      CV_8UC4);  // 8004模式使用线性算法生成的rgba图像，使用256伪彩，cv使用bgra格式，所以存在通道偏差。
                 // In 8004 mode, generate rgba image via linear algorithm using
                 // 256 pesudo color, cv using bgra format, there is channel
                 // deviation
  cv::Mat rgbImgP(
      IMAGEHEIGHT - 4, IMAGEWIDTH,
      CV_8UC4);  // 8004模式使用线性算法生成的rgba图像，使用448伪彩，cv使用bgra格式，所以存在通道偏差。
                 // 8004 mode generate rgba image via linear algorithm using 448
                 // pesudo color, cv using bgra format, there is channel
                 // deviation
  cv::Mat yuvImg;  // 8005模式输出的yuyv图像。 8005 mode output yuyv image.
  yuvImg.create(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC2);

  //测温相关参数，详见thermometry.h    Refer to thermometry.h for relevant
  // parameters
  int rangeMode = 120;
  float floatFpaTmp;
  float correction;
  float Refltmp;
  float Airtmp;
  float humi;
  float emiss;
  unsigned short distance;
  //以下cameraLens为68：1、输出为384×292，镜头=<6.8mm
  // 2、输出为256×196，镜头=<4mm.   其他默认130 The following cameraLens is 68:
  // 1,the output is 384×292, lens =<6.8mm 2, the output is 256×196, lens =<4mm.
  // Other default 130
  int cameraLens = 130;
  float shutterFix = 0;
  // end -测温相关参数

  char sn[32];                 // camera序列码   camera serial number
  char cameraSoftVersion[16];  // camera软件版本   camera Software Version
  unsigned short shutTemper;
  float floatShutTemper;  //快门温度    Shutter Temperature
  unsigned short coreTemper;
  float floatCoreTemper;  //外壳温度   Shell temperature
  const unsigned char* paletteIronRainbow =
      getPalette(0);                              // 256*3 铁虹   Iron Rainbow
  const unsigned char* palette3 = getPalette(1);  // 256*3 彩虹1    Rainbow 1
  const unsigned char* paletteRainbow = getPalette(2);  // 224*3 彩虹2 Rainbow 2
  const unsigned char* paletteHighRainbow =
      getPalette(3);  // 448*3 高动态彩虹   HDR rainbow
  const unsigned char* paletteHighContrast =
      getPalette(4);  // 448*3 高对比彩虹    High Contrast rainbow

  uint64_t time_recorder_old = uint64_t(0);
  uint64_t time_recorder_now = uint64_t(0);
  while (h.ok()) {
    for (n_buffers = 0; n_buffers < 4; n_buffers++) {
      t = (double)cvGetTickCount();
      //出队  out of queue
      buf.index = n_buffers;
      ioctl(fd, VIDIOC_DQBUF, &buf);
      delayy++;
      printf("delayy:%d\n", delayy);
      //确定数据是否完整，不完整直接抛掉  Determine if the data is complete, and
      // discard the incomplete data
      // printf("buf bytesused:%d",buf.bytesused);
      if (buf.bytesused != IMAGEHEIGHT * IMAGEWIDTH * 2) {
        //入队循环 enqueueing cycle
        ioctl(fd, VIDIOC_QBUF, &buf);
        printf("throw err data\n");
        break;
      }
      if (delayy == 51 || delayy == 52 ||
          delayy == 53)  //设置参数后下一帧传上来的参数可能不完整，抛掉 The
                         // parameters passed in the next frame after setting
                         // the parameters may not be complete. Throw them away
      {
        //入队循环 enqueueing cycle
        ioctl(fd, VIDIOC_QBUF, &buf);
        printf("throw data\n");
        break;
      }

      if (delayy == 126 || delayy == 127 ||
          delayy == 128)  //保存参数后下一帧传上来的参数可能不完整，抛掉The
                          // parameters passed in the next frame after saving
                          // the parameters may not be complete. Throw them away
      {
        //入队循环 enqueueing cycle
        ioctl(fd, VIDIOC_QBUF, &buf);
        printf("throw data\n");
        break;
      }

      //查看采集数据的时间戳之差，单位为微妙   Check the timestamp of data
      // capturing, unit: microsecond
      buffers[n_buffers].timestamp =
          buf.timestamp.tv_sec * 1000000 + buf.timestamp.tv_usec;
      t3s_thermal_msg.timestamp_t3s_sec = uint32_t(buf.timestamp.tv_sec);
      t3s_thermal_msg.timestamp_t3s_usec = uint32_t(buf.timestamp.tv_usec);
      cur_time = buffers[n_buffers].timestamp;
      extra_time = cur_time - last_time;
      last_time = cur_time;
      time_recorder_now = ros::WallTime::now().toNSec();
      printf("time duration from after_pub to before_temp_processing:%llu\n",
             time_recorder_now - time_recorder_old);
      time_recorder_old = time_recorder_now;
      // printf("cur_time_deta:%lld\n\n",cur_time);
      // printf("time_deta:%lld\n\n",extra_time);
      // printf("buf_len:%d\n",buffers[n_buffers].length);

      unsigned short* orgData = (unsigned short*)buffers[n_buffers].start;
      unsigned short* fourLinePara =
          orgData +
          IMAGEWIDTH *
              (IMAGEHEIGHT - 4);  //后四行参数  Parameters in last four lines

      int amountPixels = 0;
      switch (IMAGEWIDTH) {
        case 384:
          amountPixels = IMAGEWIDTH * (4 - 1);
          break;
        case 240:
          amountPixels = IMAGEWIDTH * (4 - 3);
          break;
        case 256:
          amountPixels = IMAGEWIDTH * (4 - 3);
          break;
        case 640:
          amountPixels = IMAGEWIDTH * (4 - 1);
          break;
      }
      memcpy(&shutTemper, fourLinePara + amountPixels + 1,
             sizeof(unsigned short));
      // printf("cpyPara  shutTemper:%d ",shutTemper);
      floatShutTemper =
          shutTemper / 10.0f - 273.15f;  //快门温度  Shutter Temperature
      memcpy(&coreTemper, fourLinePara + amountPixels + 2,
             sizeof(unsigned short));  //外壳  Shell temperature
      // printf("cpyPara  coreTemper:%d ",coreTemper);
      floatCoreTemper = coreTemper / 10.0f - 273.15f;
      printf("cpyPara  floatShutTemper:%f,floatCoreTemper:%f,floatFpaTmp:%f\n",
             floatShutTemper, floatCoreTemper, floatFpaTmp);
      memcpy((unsigned short*)cameraSoftVersion,
             fourLinePara + amountPixels + 24,
             16 * sizeof(uint8_t));  // camera soft version
      printf("cameraSoftVersion:%s\n", cameraSoftVersion);
      memcpy((unsigned short*)sn, fourLinePara + amountPixels + 32,
             32 * sizeof(uint8_t));  // SN
      printf("sn:%s\n", sn);
      int userArea = amountPixels + 127;
      memcpy(&correction, fourLinePara + userArea,
             sizeof(float));  //修正  Correction
      userArea = userArea + 2;
      memcpy(&Refltmp, fourLinePara + userArea,
             sizeof(float));  //反射温度   Reflection temperature
      userArea = userArea + 2;
      memcpy(&Airtmp, fourLinePara + userArea,
             sizeof(float));  //环境温度    Ambient temperature
      userArea = userArea + 2;
      memcpy(&humi, fourLinePara + userArea, sizeof(float));  //湿度   Humidity
      userArea = userArea + 2;
      memcpy(&emiss, fourLinePara + userArea,
             sizeof(float));  //发射率   Emissivity
      userArea = userArea + 2;
      memcpy(&distance, fourLinePara + userArea,
             sizeof(unsigned short));  //距离   Distance
      printf(
          "Airtmp:%f,correction:%f,distance:%d,emiss:%f,Refltmp:%f,humi:%f\n",
          Airtmp, correction, distance, emiss, Refltmp, humi);

      // printf("delayy:%d\n",delayy);
      if (delayy % 4500 ==
          30)  //每三分钟打一次快门，人体高精度版本每分钟打一次   Shutter
               // calibrates once every three minutes, for body temperature
               // measuring, once per minute
      {
        if (v4l2_control(0x8000) == FALSE) {
          printf("shutter fail~~\n");
        }
      }
      if (delayy % 4500 ==
          25)  //打快门前5帧重新计算table，打快门时不可计算表，数据可能有错误
               // Recalculate the table during the five frames before shutter
               // calibrates, otherwise may be errors
      {
        /*thermometryT(           IMAGEWIDTH,
                            IMAGEHEIGHT,
                            temperatureTable,
                            orgData,
                            &floatFpaTmp,
                            & correction,
                            & Refltmp,
                            & Airtmp,
                            & humi,
                            & emiss,
                            & distance,
                            cameraLens,
                            shutterFix
                            rangeMode);*/
        //用后四行参数来计算表   Calculate table with last four lines
        thermometryT4Line(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable,
                          fourLinePara, &floatFpaTmp, &correction, &Refltmp,
                          &Airtmp, &humi, &emiss, &distance, cameraLens,
                          shutterFix, rangeMode);
        if (delayy > 9000) {
          delayy = 0;
        }
      }

      // if(delayy%25==10)//一般一秒钟查一次表确定对应点的温度，首次查表要计算完table
      // Usually search temperature of points with table every second. Calculate
      // the whole table when first time searching.
      // {

      /*temperatureData[0]=centerTmp;
      temperatureData[1]=(float)maxx1;
      temperatureData[2]=(float)maxy1;
      temperatureData[3]=maxTmp;
      temperatureData[4]=(float)minx1;
      temperatureData[5]=(float)miny1;
      temperatureData[6]=minTmp;
      temperatureData[7]=point1Tmp;
      temperatureData[8]=point2Tmp;
      temperatureData[9]=point3Tmp;*/
      //根据8004或者8005模式来查表，8005模式下仅输出以上注释的10个参数，8004模式下数据以上参数+全局温度数据
      // Search on the table with 8004 or 8005 mode, 8005 mode only outputs the
      // 10 parameters above, 8004 mode include above parameters with overall
      // temperature data
      thermometrySearch(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable, orgData,
                        temperatureData, rangeMode, OUTPUTMODE);
      printf(
          "centerTmp:%.2f,maxTmp:%.2f,minTmp:%.2f,avgTmp:%.2f,point1Tmp:%.2f\n",
          temperatureData[0], temperatureData[3], temperatureData[6],
          temperatureData[9], temperatureData[7]);

      /**
       * 线性图像算法 linear algorithm
       * 图像效果不及专业级算法，但是处理效率快，对主频几乎没要求
       * Poor images than professional algorithm, but runs faster and no
       * requirement in basic frequency
       *
       */

      amountPixels = IMAGEWIDTH * (IMAGEHEIGHT - 4);

      unsigned short detectAvg = orgData[amountPixels];
      // printf("cpyPara  detectAvg:%d ",detectAvg);
      amountPixels++;
      unsigned short fpaTmp = orgData[amountPixels];
      amountPixels++;
      unsigned short maxx1 = orgData[amountPixels];
      amountPixels++;
      unsigned short maxy1 = orgData[amountPixels];
      amountPixels++;
      unsigned short max = orgData[amountPixels];
      // printf("cpyPara  max:%d ",max);
      amountPixels++;
      unsigned short minx1 = orgData[amountPixels];
      amountPixels++;
      unsigned short miny1 = orgData[amountPixels];
      amountPixels++;
      unsigned short min = orgData[amountPixels];
      amountPixels++;
      unsigned short avg = orgData[amountPixels];
      // printf("cpyPara  avg:%hx",orgData[10]);

      unsigned char* orgOutput = orgRgbImgP.data;
      int ro = (max - min) > 0 ? (max - min) : 1;
      int avgSubMin = (avg - min) > 0 ? (avg - min) : 1;
      int maxSubAvg = (max - avg) > 0 ? (max - avg) : 1;
      int ro1 = (avg - min) > 97 ? 97 : (avg - min);
      int ro2 = (max - avg) > 157 ? 157 : (max - avg);
      // int ga=(max - min)>254?254:(max - min);
      for (int i = 0; i < IMAGEHEIGHT - 4; i++) {
        for (int j = 0; j < IMAGEWIDTH; j++) {
          // printf("i:%d,j:%d\n",i,j);
          //黑白：灰度值0-254单通道。
          // paletteIronRainbow：（0-254）×3三通道。两个都是255，所以使用254
          // Black&WHite: Grayscale (0-254), single channel.  paletteIronRainbow
          // (0-254), triple channels. Both are 255, so take 254
          int gray = 0;
          if (orgData[i * IMAGEWIDTH + j] > avg) {
            gray = (int)(ro2 * (orgData[i * IMAGEWIDTH + j] - avg) / maxSubAvg +
                         97);
          } else {
            gray = (int)(ro1 * (orgData[i * IMAGEWIDTH + j] - avg) / avgSubMin +
                         97);
          }
          orgLumiImgL.at<uchar>(i, j) = (uchar)gray;
          int intGray = (int)gray;
          int paletteNum = 3 * intGray;
          orgOutput[4 * (i * IMAGEWIDTH + j)] =
              (unsigned char)paletteIronRainbow[paletteNum + 2];
          orgOutput[4 * (i * IMAGEWIDTH + j) + 1] =
              (unsigned char)paletteIronRainbow[paletteNum + 1];
          orgOutput[4 * (i * IMAGEWIDTH + j) + 2] =
              (unsigned char)paletteIronRainbow[paletteNum];
          orgOutput[4 * (i * IMAGEWIDTH + j) + 3] = 1;
        }
      }
      cv::imshow("orgLumiImgL", orgLumiImgL);
      // convert this image to sensor_msgs::Image
      // and publish  it
      // and record it
      // ***********************ywy 1st edition****************************
      // ***********************ywy 1st edition****************************
      // ***********************ywy 1st edition****************************
      // cv_bridge::CvImagePtr cv_ptr;
      // cv_ptr->image = orgRgbImgP;
      // ywy_thermal_img_pub.publish(cv_ptr->toImageMsg());

      // ************************yyy 2nd edition *************************
      //   sensor_msgs::ImagePtr msg1 =
      //       cv_bridge::CvImage(std_msgs::Header(), "mono8", orgRgbImgP)
      //           .toImageMsg();
      //   ywy_thermal_img_pub.publish(msg1);

      // ************************yyy 3rd edition**************************
      //   cv_bridge::CvImage img_bridge;
      //   sensor_msgs::Image img_msg;  // >> message to be sent

      //   std_msgs::Header header;          // empty header
      // //   header.seq = counter;             // user defined counter
      //   header.stamp = ros::Time::now();  // time
      // //   img_bridge = cv_bridge::CvImage(
      // //       header, sensor_msgs::image_encodings::RGB8, orgLumiImgL);
      //   img_bridge = cv_bridge::CvImage(
      //       header, sensor_msgs::image_encodings::TYPE_8UC1, orgLumiImgL);
      //   img_bridge.toImageMsg(img_msg);
      // from cv_bridge to sensor_msgs::Image
      //   pub_img.publish(
      //       img_msg);  // ros::Publisher pub_img =
      //                  // node.advertise<sensor_msgs::Image>("topic",
      //                  queuesize);

      // ************************yyy 4th edition**************************
      sensor_msgs::ImagePtr fourth_thermal_img_msg(new sensor_msgs::Image);
      //   4th_thermal_img_msg->header.seq = head_sequence_id;
      fourth_thermal_img_msg->header.stamp = ros::Time::now();
      fourth_thermal_img_msg->header.frame_id = 'world';
      fourth_thermal_img_msg->width = orgLumiImgL.cols;
      fourth_thermal_img_msg->height = orgLumiImgL.rows;
      fourth_thermal_img_msg->encoding = sensor_msgs::image_encodings::MONO8;
      fourth_thermal_img_msg->is_bigendian = 0;
      int step = sizeof(unsigned char) * fourth_thermal_img_msg->width;
      int size = step * fourth_thermal_img_msg->height;
      fourth_thermal_img_msg->step = step;
      fourth_thermal_img_msg->data.resize(size);
      memcpy(&fourth_thermal_img_msg->data[0], orgLumiImgL.data, size);
      pub_img.publish(fourth_thermal_img_msg);
      // ********************************************************************
      // ********************************************************************
      // ********************************************************************
      cv::imshow("orgRgbImgP", orgRgbImgP);
      //   cv::imwrite("orgRgbImgP.jpg", orgRgbImgP);

      // cloud9***********
      time_recorder_now = ros::WallTime::now().toNSec();
      printf(
          "time duration from after_temp_processing to before_temp_pub:%llu\n",
          time_recorder_now - time_recorder_old);
      time_recorder_old = time_recorder_now;
      // t3s_thermal_msg.temperaturedata.push_back(temperatureData[0]);
      // t3s_thermal_msg.temperaturedata.push_back(temperatureData[1]);
      // t3s_thermal_msg.temperaturedata.push_back(temperatureData[2]);
      // t3s_thermal_msg.temperaturedata.push_back(temperatureData[3]);

      // very important!!!need to empty the msg data before using push_back
      // function. Or the the new data will contain old data.
      t3s_thermal_msg.temperaturedata.clear();
      printf("msg_temperaturedata_size%d\n",
             t3s_thermal_msg.temperaturedata.size());
      for (i = 0; i < (10 + IMAGEWIDTH * (IMAGEHEIGHT - 4)); i++) {
        t3s_thermal_msg.temperaturedata.push_back(temperatureData[i]);
      }
      // t3s_thermal_msg.temperaturedata = 15.0;
      t3s_thermal_msg.image_height = IMAGEHEIGHT - 4;
      t3s_thermal_msg.image_width = IMAGEWIDTH;

      t3s_thermal_inf_pub.publish(t3s_thermal_msg);
      time_recorder_now = ros::WallTime::now().toNSec();
      printf("time duration from before_temp_pub to after_temp_pub:%llu\n",
             time_recorder_now - time_recorder_old);
      time_recorder_old = time_recorder_now;
      ros::spinOnce();
      looprate.sleep();
      //***********cloud9

      // orgOutput = rgbImgP.data;
      // ro = (max - min)>0?(max - min):1;
      // avgSubMin=(avg-min)>0?(avg-min):1;
      // maxSubAvg=(max-avg)>0?(max-avg):1;
      // ro1=(avg-min)>170?170:(avg-min);
      // ro2=(max-avg)>276?276:(max-avg);
      // for(int i=0; i<IMAGEHEIGHT-4; i++)
      // {
      //     for(int j=0; j<IMAGEWIDTH; j++)
      //     {
      //         //printf("i:%d,j:%d\n",i,j);
      //         // paletteHighContrast（0-448）×3三通道，所以使用447  triple
      //         channels, so take 447 int gray=0;
      //         if(orgData[i*IMAGEWIDTH+j]>avg)
      //         {
      //             gray =
      //             (int)(ro2*(orgData[i*IMAGEWIDTH+j]-avg)/maxSubAvg+170);
      //         }
      //         else
      //         {
      //             gray =
      //             (int)(ro1*(orgData[i*IMAGEWIDTH+j]-avg)/avgSubMin+170);
      //         }
      //         orgLumiImgL.at<uchar>(i,j) = (uchar)gray;
      //         int intGray=(int)gray;
      //         int paletteNum=3*intGray;

      //         orgOutput[4*(i*IMAGEWIDTH+j)]=(unsigned
      //         char)paletteHighContrast[paletteNum+2];
      //         orgOutput[4*(i*IMAGEWIDTH+j)+1]=(unsigned
      //         char)paletteHighContrast[paletteNum+1];
      //         orgOutput[4*(i*IMAGEWIDTH+j)+2]=(unsigned
      //         char)paletteHighContrast[paletteNum];
      //         orgOutput[4*(i*IMAGEWIDTH+j)+3]=1;
      //     }
      // }
      // cv::imshow("rgbImgP", rgbImgP);
      // //end-线性算法

      //输出以上注释的10个参数，跟8004或者8005模式无关   Output 10 parameters
      // above, not relevant to modes 8004 or 8005
      /*float* temperatureData2=(float*)calloc(10,sizeof(float));
      thermometrySearchCMM(IMAGEWIDTH,IMAGEHEIGHT,temperatureTable,fourLinePara,temperatureData2,rangeMode);
              printf("centerTmp:%.2f,maxx1:%.2f,maxy1:%.2f\n",temperatureData2[0],temperatureData2[1],temperatureData2[2]);
                           free(temperatureData2);*/

      //指定要查询的个数及数据位置，仅可输入8004模式下的数据   Set the count of
      // query and data address, only for data in 8004 mode
      /*int count=20;
      unsigned short* queryData=orgData+IMAGEWIDTH*(IMAGEHEIGHT-4)/2;
      float* temperatureData3=(float*)calloc(count,sizeof(float));
      thermometrySearchSingle(IMAGEWIDTH,IMAGEHEIGHT,temperatureTable,fourLinePara,count,queryData,temperatureData3,rangeMode);
      printf("temperatureData3[0]:%.2f,temperatureData3[1]:%.2f,temperatureData3[2]:%.2f\n",temperatureData3[0],temperatureData3[1],temperatureData3[2]);
                           free(temperatureData3);*/

      //}

      if (delayy == 50)  //设置参数  Setting parameter
      {
        // sendCorrection(2.1f);
        // sendReflection(26.0f);
        // sendAmb(26.0f);
        // sendHumidity(0.47f);
        // sendEmissivity(0.80f);
        sendDistance(3);
      }
      if (delayy == 60) {
        setPoint(IMAGEWIDTH / 2, (IMAGEHEIGHT - 4) / 2, 0);
      }

      if (delayy == 110)  //打快门  Shutter calibrates
      {
        if (v4l2_control(0x8000) == FALSE) {
          printf("shutter fail~~\n");
        }
      }
      if (delayy == 120)  //使用新的参数重新计算表   Calculate on table with new
                          // parameters
      {
        //用后四行参数来计算表    Calculate on table with last four lines of
        // parameters
        thermometryT4Line(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable,
                          fourLinePara, &floatFpaTmp, &correction, &Refltmp,
                          &Airtmp, &humi, &emiss, &distance, cameraLens,
                          shutterFix, rangeMode);
      }
      if (delayy == 125)  //保存参数   Save parameters
      {
        savePara();
        printf("savePara\n");
      }

      //以下用于8005模式中yuyv输出转bgr，使用的是opencv
      // Following codes for converting yuyv outputs into bgr in 8005 mode,
      // using OpenCV
      if (OUTPUTMODE == 5) {
        memcpy(yuvImg.data, (unsigned char*)orgData,
               (IMAGEHEIGHT - 4) * 2 * IMAGEWIDTH * sizeof(unsigned char));
        cv::cvtColor(yuvImg, rgbImg, cv::COLOR_YUV2RGB_YUYV);
      }

      //以下用于8004模式中计算出rgba来显示，opencv使用bgra来显示，故而通道有差异。
      // Following codes for computing rgba outputs in 8004 mode; using OpenCV
      // to show bgra, so there are differences in channel
      // if(OUTPUTMODE==4)
      // {
      //     Compute(orgData,rgbImg.data,0,irBuffers[n_buffers].midVar);//0-6 to
      //     change platte
      // }
      // cv::imshow("rgbImg", rgbImg);

      // printf("VIDIOC_DQBUF~~\n");
      ioctl(fd, VIDIOC_QBUF, &buf);
      //在 driver 内部管理着两个 buffer queues ，一个输入队列，一个输出队列。
      //对于 capture device 来说，当输入队列中的 buffer
      //被塞满数据以后会自动变为输出队列，
      // There are two buffer queues inside driver: input queues and output
      // queues For capture device, when buffer in input queue is fulled by
      // data, it will transfer to output queue printf("VIDIOC_QBUF~~\n");
      // //等待调用 VIDIOC_DQBUF 将数据进行处理以后  Calling VIDIOC_DQBUF after
      // data processed 重新调用 VIDIOC_QBUF 将 buffer 重新放进输入队列. Calling
      // VIDIOC_DQBUF again, place buffer into output queue
      t = (double)cvGetTickCount() - t;
      // printf("used time is %gms\n",(t/(cvGetTickFrequency()*1000)));
      if ((cv::waitKey(1) & 255) == 27) exit(0);
    }
  }

  for (n_buffers = 0; n_buffers < 4; n_buffers++) {
    //释放专业图像算法占用的资源   Release buffers captured by professional
    // image algorithm
    if (OUTPUTMODE == 4) {
      SimplePictureProcessingDeinit();
      if (irBuffers[n_buffers].midVar != NULL) {
        SimplePictureProcessingDeinitMidVar(irBuffers[n_buffers].midVar);
        free(irBuffers[n_buffers].midVar);
        irBuffers[n_buffers].midVar = NULL;
      }
    }
    if (irBuffers[n_buffers].destBuffer != NULL) {
      free(irBuffers[n_buffers].destBuffer);
      irBuffers[n_buffers].destBuffer = NULL;
    }
    // end -释放专业图像算法占用的资源   Release buffers

    if (temperatureData != NULL) {
      free(temperatureData);
      temperatureData = NULL;
    }
  }

  v4l2_release();  // 停止视频采集命令，应用程序调用VIDIOC_
                   // STREAMOFF停止视频采集命令后，视频设备驱动程序不在采集视频数据。
                   // Calling VIDIOC_ STREAMOFF to stop capturing video, video
                   // device driver stops capturing video
  return 0;
}
int v4l2_release() {
  unsigned int n_buffers;
  enum v4l2_buf_type type;

  //关闭流  Video stream OFF
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(fd, VIDIOC_STREAMOFF, &type);

  //关闭内存映射   Memory map OFF
  for (n_buffers = 0; n_buffers < 4; n_buffers++) {
    munmap(buffers[n_buffers].start, buffers[n_buffers].length);
  }

  //释放自己申请的内存   Free buffers
  free(buffers);

  //关闭设备   Device OFF
  close(fd);
  return TRUE;
}

int init_v4l2(string videoX) {
  const char* videoXConst = videoX.c_str();
  if ((fd = open(videoXConst, O_RDWR)) == -1)  //打开video1  Open Video1
  {
    printf("Opening video device error\n");
    return FALSE;
  }
  if (ioctl(fd, VIDIOC_QUERYCAP, &cap) ==
      -1)  // 查询视频设备的功能   Query capabilities of video device
  {
    printf("unable Querying Capabilities\n");
    return FALSE;
  } else

  {
    printf(
        "Driver Caps:\n"
        "  Driver: \"%s\"\n"
        "  Card: \"%s\"\n"
        "  Bus: \"%s\"\n"
        "  Version: %d\n"
        "  Capabilities: %x\n",
        cap.driver, cap.card, cap.bus_info, cap.version, cap.capabilities);
    string str = "";
    str = (char*)cap.card;
    /*if(!str.find("T3S")){
            close(fd);
            return FALSE;
    }*/
  }
  /* if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ==
   V4L2_CAP_VIDEO_CAPTURE){//是否支持V4L2_CAP_VIDEO_CAPTURE printf("Camera
   device %s: support capture\n",FILE_VIDEO1);
   }
   if((cap.capabilities & V4L2_CAP_STREAMING) ==
   V4L2_CAP_STREAMING){//是否支持V4L2_CAP_STREAMING printf("Camera device %s:
   support streaming.\n",FILE_VIDEO1);
   }
  */
  fmtdesc.index = 0;
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  printf("Support format: \n");
  while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) !=
         -1)  // 获取当前视频设备支持的视频格式   Capture current video format
              // of the device
  {
    printf("\t%d. %s\n", fmtdesc.index + 1, fmtdesc.description);
    fmtdesc.index++;
  }
  // set fmt
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = IMAGEWIDTH;
  fmt.fmt.pix.height = IMAGEHEIGHT;
  fmt.fmt.pix.pixelformat =
      V4L2_PIX_FMT_YUYV;  //使用V4L2_PIX_FMT_YUYV  Calling V4L2_PIX_FMT_YUYV
  // fmt.fmt.pix.field = V4L2_FIELD_NONE;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) ==
      -1)  // 设置视频设备的视频数据格式，例如设置视频图像数据的长、宽，图像格式（JPEG、YUYV格式）
  {  // Set video data format, such as Width, Height of image, format (JPEG,
     // YUYV etc)
    printf("Setting Pixel Format error\n");
    return FALSE;
  }
  if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1)  //获取图像格式   Get video format
  {
    printf("Unable to get format\n");
    return FALSE;
  }
  IMAGEWIDTH = fmt.fmt.pix.width;    //更正宽  fix Width
  IMAGEHEIGHT = fmt.fmt.pix.height;  //更正高   fix Height
  printf("IMAGEWIDTH:%d,IMAGEHEIGHT:%d\n", IMAGEWIDTH, IMAGEHEIGHT);
  memset(&stream_para, 0, sizeof(struct v4l2_streamparm));
  stream_para.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  stream_para.parm.capture.timeperframe.denominator = 25;
  stream_para.parm.capture.timeperframe.numerator = 1;

  if (ioctl(fd, VIDIOC_S_PARM, &stream_para) == -1) {
    printf("Unable to set frame rate\n");
    return FALSE;
  }
  if (ioctl(fd, VIDIOC_G_PARM, &stream_para) == -1) {
    printf("Unable to get frame rate\n");
    return FALSE;
  }
  {
    printf("numerator:%d\ndenominator:%d\n",
           stream_para.parm.capture.timeperframe.numerator,
           stream_para.parm.capture.timeperframe.denominator);
  }
  //        else

  /*        {
              printf("fmt.type:\t%d\n",fmt.type);         //可以输出图像的格式
     Output image format
              printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat &
     0xFF,(fmt.fmt.pix.pixelformat >> 8) & 0xFF,\ (fmt.fmt.pix.pixelformat >>
     16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
              printf("pix.height:\t%d\n",fmt.fmt.pix.height);
              printf("pix.field:\t%d\n",fmt.fmt.pix.field);
          }
  */
  return TRUE;
}

int v4l2_grab(void) {
  //-------cloud9--test------
  // printf("---------------cloud9_v4l2_test-------------------");

  // struct v4l2_requestbuffers req = {0};
  // 4  request for 4buffers 缓存不可少于两个   count of buffer >= 2
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) ==
      -1)  //开启内存映射或用户指针I/O    Start memory map or pointer I/O
  {
    printf("Requesting Buffer error\n");
    return FALSE;
  }
  // 5 mmap for buffers
  buffers = (buffer*)malloc(req.count * sizeof(*buffers));
  if (!buffers) {
    printf("Out of memory\n");
    return FALSE;
  }
  unsigned int n_buffers;
  for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
    // struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) ==
        -1)  // 查询已经分配的V4L2的视频缓冲区的相关信息，包括视频缓冲区的使用状态、
    {
      //在内核空间的偏移地址、缓冲区长度等。 Search info of allocated V4L2 video
      // buffer, including status, offset address,buffer length etc
      printf("Querying Buffer error\n");
      return FALSE;
    }
    buffers[n_buffers].length = buf.length;

    buffers[n_buffers].start = (unsigned char*)mmap(
        NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

    if (buffers[n_buffers].start == MAP_FAILED) {
      printf("buffer map error\n");
      return FALSE;
    }
  }
  // 6 queue
  for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
    buf.index = n_buffers;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_QBUF,
              &buf))  // 投放一个空的视频缓冲区到视频缓冲区输入队列中  Put in
                      // empty video buffer to input queue of video buffer
    {
      printf("query buffer error\n");
      return FALSE;
    }
  }
  // 7 starting
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) == -1)  //
  {
    printf("stream on error\n");
    return FALSE;
  }
  return TRUE;
}
int v4l2_control(int value) {
  ctrl.id = V4L2_CID_ZOOM_ABSOLUTE;
  ctrl.value = value;  // change output mode 0x8004/0x8005
  // shutter 0x8000
  if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
    printf("v4l2_control error\n");
    return FALSE;
  }
  return TRUE;
}
int traversalVideo(void) {
  string device = "/dev";
  string video = "video";
  char* KEY_PTR = (char*)video.c_str();
  char* FILE_PTR = (char*)device.c_str();

  DIR* dir;
  struct dirent* ptr;
  char base[1000];

  if ((dir = opendir(FILE_PTR)) == NULL) {
    perror("Open dir error...");
    exit(1);
  }

  while ((ptr = readdir(dir)) != NULL) {
    string name = "";
    name = (char*)ptr->d_name;
    if (name.find("video") != string::npos) {
      string allName = device + "/" + name;
      if (init_v4l2(allName)) {
        closedir(dir);
        return 1;
      }
    }
    printf("d_name:%s/%s\n", FILE_PTR, ptr->d_name);
  }

  closedir(dir);
  return 0;
}

void sendFloatCommand(int position, unsigned char value0, unsigned char value1,
                      unsigned char value2, unsigned char value3, int interval0,
                      int interval1, int interval2, int interval3,
                      int interval4) {
  int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
  printf("psitionAndValue0:%X\n", psitionAndValue0);
  // v4l2_control(psitionAndValue0);
  if (v4l2_control(psitionAndValue0) == FALSE) {
    printf("control fail psitionAndValue0~~\n");
    exit(EXIT_FAILURE);
  }
  int psitionAndValue1 = ((position + 1) << 8) | (0x000000ff & value1);
  printf("psitionAndValue1:%X\n", psitionAndValue1);
  if (v4l2_control(psitionAndValue1) == FALSE) {
    printf("control fail psitionAndValue1~~\n");
    exit(EXIT_FAILURE);
  }
  int psitionAndValue2 = ((position + 2) << 8) | (0x000000ff & value2);
  printf("psitionAndValue2:%X\n", psitionAndValue2);
  if (v4l2_control(psitionAndValue2) == FALSE) {
    printf("control fail psitionAndValue2~~\n");
    exit(EXIT_FAILURE);
  }
  int psitionAndValue3 = ((position + 3) << 8) | (0x000000ff & value3);
  printf("psitionAndValue3:%X\n", psitionAndValue3);
  if (v4l2_control(psitionAndValue3) == FALSE) {
    printf("control fail psitionAndValue3~~\n");
    exit(EXIT_FAILURE);
  }
}
void sendUshortCommand(int position, unsigned char value0,
                       unsigned char value1) {
  int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
  printf("psitionAndValue0:%X\n", psitionAndValue0);
  // v4l2_control(psitionAndValue0);
  if (v4l2_control(psitionAndValue0) == FALSE) {
    printf("control fail psitionAndValue0~~\n");
    exit(EXIT_FAILURE);
  }
  int psitionAndValue1 = ((position + 1) << 8) | (0x000000ff & value1);
  printf("psitionAndValue1:%X\n", psitionAndValue1);
  if (v4l2_control(psitionAndValue1) == FALSE) {
    printf("control fail psitionAndValue1~~\n");
    exit(EXIT_FAILURE);
  }
}
void sendByteCommand(int position, unsigned char value0, int interval0) {
  int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
  v4l2_control(psitionAndValue0);
}

void sendCorrection(float correction) {
  unsigned char iputCo[4];
  memcpy(iputCo, &correction, sizeof(float));
  sendFloatCommand(0 * 4, iputCo[0], iputCo[1], iputCo[2], iputCo[3], 20, 40,
                   60, 80, 120);
  printf("sendCorrection 0:%d,1:%d,2:%d,3:%d\n", iputCo[0], iputCo[1],
         iputCo[2], iputCo[3]);
}
void sendReflection(float reflection) {
  unsigned char iputRe[4];
  memcpy(iputRe, &reflection, sizeof(float));
  sendFloatCommand(1 * 4, iputRe[0], iputRe[1], iputRe[2], iputRe[3], 20, 40,
                   60, 80, 120);
}
void sendAmb(float amb) {
  unsigned char iputAm[4];
  memcpy(iputAm, &amb, sizeof(float));
  sendFloatCommand(2 * 4, iputAm[0], iputAm[1], iputAm[2], iputAm[3], 20, 40,
                   60, 80, 120);
}
void sendHumidity(float humidity) {
  unsigned char iputHu[4];
  memcpy(iputHu, &humidity, sizeof(float));
  sendFloatCommand(3 * 4, iputHu[0], iputHu[1], iputHu[2], iputHu[3], 20, 40,
                   60, 80, 120);
}
void sendEmissivity(float emiss) {
  unsigned char iputEm[4];
  memcpy(iputEm, &emiss, sizeof(float));
  sendFloatCommand(4 * 4, iputEm[0], iputEm[1], iputEm[2], iputEm[3], 20, 40,
                   60, 80, 120);
}

void sendDistance(unsigned short distance) {
  unsigned char iputDi[2];
  memcpy(iputDi, &distance, sizeof(unsigned short));
  sendUshortCommand(5 * 4, iputDi[0], iputDi[1]);
}
void savePara() { v4l2_control(0x80ff); }
void setPoint(int viewX1, int viewY1, int indexOfPoint) {
  int x1, y1;
  switch (indexOfPoint) {
    case 0:
      x1 = 0xf000 + viewX1;
      y1 = 0xf200 + viewY1;
      break;
    case 1:
      x1 = 0xf400 + viewX1;
      y1 = 0xf600 + viewY1;
      break;
    case 2:
      x1 = 0xf800 + viewX1;
      y1 = 0xfa00 + viewY1;
      break;
  }
  v4l2_control(x1);
  v4l2_control(y1);
}

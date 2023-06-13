#ifndef SIMPLE_PICTURE_PROCESSING_H
#define SIMPLE_PICTURE_PROCESSING_H



#include<stdio.h>
#include<stdlib.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C"  //C++
{
#endif
    void SimplePictureProcessingInit(int width,int height);//初始化Initialize
    void SimplePictureProcessingInitMidVar(size_t** midVar);//初始化中间量initialize Mid Variables
    void SetParameter(float a,float b,float c,float d,float e,float f);
	//设置参数，参数已经调校好，请勿修改 Set parameters, all set, please dont modify

	 /**
	 * 使用专业图像算法将8004数据转变为图像 Transfer 8004 data into graphs via algorithm
	 * @para input：输入指针，指向8004数据开始 Input pointer, point to 8004 data
	 * @para output：输出指针，指向rgba图像开始 Output pointer, point to rgba graph
	 * @para kindOfPalette1: 0：白热。White Hot 1：黑热。Black Hot 2：铁虹。 Iron Raindow 3：彩虹1。Rainbow 1 
	 * 4、彩虹2. Rainbow 2 5:高对比彩虹1.High-contrast rainbow 1 6:高对比彩虹2. High-contrast rainbow 2 >=7、用户色板 Customize palette
	 */
    void Compute(unsigned short* input,unsigned char* output,int kindOfPalette1,size_t** midVar);

	 /**
	 * 设置用户色板 Set User Palette
	 * @para palette：输入指针，指向用户色板开始 input pointer, point to user palette
	 * @para typeOfPalette：需要>=7 needs >=7
	 */
    void SetUserPalette(unsigned char* palette,int typeOfPalette);
    void SimplePictureProcessingDeinit();//释放资源 Release resources 
    void SimplePictureProcessingDeinitMidVar(size_t** midVar);//释放中间变量 Release mid variables

	/**
	 * 获得色板Capture palette
	 * @para type：
	 * (0)：256*3 铁虹 iorn rainbow
	 * (1)：256*3 彩虹1 Rainbow 1
	 * (2)：224*3 彩虹2 Rainbow 2
	 * (3)：448*3 高对比彩虹1 High-Contrast Rainbow 1
	 * (4)：448*3 高对比彩虹2 High-Contrast Rainbow 2
	 */
    const unsigned char* getPalette(int type);


#ifdef __cplusplus
}
#endif

#endif

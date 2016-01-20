#pragma once
#include <stdlib.h>
#include <stdio.h>

#include "stdafx.h"

//File Format - Binary file
//Header : Data Count
//Contents :
//RGBFilePathLength(Int) RGBFilePath(char * Len) DepthFilePathLength(Int) DepthFilePath(char * Len) EndEffector(UpperLeft, UpperRight, Thumb)
//*Endeffector = 3d position ( x , y, z - float)

typedef struct SupervisedData_{
	char RGBPath[256];
	char DepthPath[256];
	cv::Point3f UpperLeft;
	cv::Point3f UpperRight;
	cv::Point3f Thumb;
}SupervisedData;

class EndEffectorRecord
{
public:
	EndEffectorRecord(void);
	~EndEffectorRecord(void);

	//write function
	void CreateRecordFile(char *fileName);
	void WriteData(char *RGBImgPath, char *DepthImgPath, cv::Point3f UpperLeft, cv::Point3f UpperRight, cv::Point3f Thumb);
	void CloseCreatedFile();

	//Read function
	int OpenRecordFile(char *fileName);
	void ReadData(cv::Mat *RGBimg, cv::Mat *DepthImg, cv::Point3f *UpperLeft, cv::Point3f *UpperRight, cv::Point3f *Thumb);			//return DataCount;
	void CloseOpendFile();

private:
	FILE *fp_;
	std::vector<SupervisedData> data_;
};


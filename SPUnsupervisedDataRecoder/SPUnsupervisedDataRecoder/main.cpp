#include <conio.h>
#include <strsafe.h>

#include "stdafx.h"

#include "KinectConnecter.h"
#include "EndEffectorRecord.h"

using namespace armsdk;

void printClassData(char* path, int *count);
bool fileTypeCheck(TCHAR *fileName);

int main(){
	printf("==================Unsupervise Data recoder===================\n");
	Mat KinectColorImage;			//Kinect Color Image
	Mat KinectDepthImage;
	Mat KinectMappingImage;

	int saveResolution = 480*2/3;
	int saveDepthResolution = 180*2/3;
	int presentStoredDataCount = 0;
	int FrameCount = 0;
	int saveCheck = -1;

	KinectColorImage.create(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC4);			//Kinect Color Image format BGRA 4 channel image
	KinectDepthImage.create(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_8UC4);			//Kinect Depth Image format BGRA 4 Channel image
	KinectMappingImage.create(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC1);

	KinectConnecter Kinect;
	EndEffectorRecord record;
	armsdk::RobotInfo robot;
	Kinematics kin;

	//Leftarm
	robot.AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 2);
	robot.AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 4);
	robot.AddJoint( 30.0,  ML_PI_2,  246.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 6);
	robot.AddJoint(-30.0, -ML_PI_2,    0.0, -ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 8);
	robot.AddJoint(  0.0,  ML_PI_2,  216.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 10);
	robot.AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 12);

	kin.setRobotInfo(&robot);

	Kinect.KinectInitialize();

	namedWindow("KinectColorFrame", CV_WINDOW_KEEPRATIO);
	namedWindow("KinectDepthFrame", CV_WINDOW_KEEPRATIO);
	//namedWindow("KinectMapFrame", CV_WINDOW_KEEPRATIO);

	while(1){
		presentStoredDataCount = 0;
		printf("\n=================Save Data form info===============\n");
		printf("Default Save iamge root : %s\n", DEFAULT_PATH);
		printf("Save resoltuion : %d x %d\n", saveResolution, saveResolution);
		printf("Transform resolution : %d x %d\n", TRANSFORM_SIZE, TRANSFORM_SIZE);
		printClassData(DEFAULT_PATH ,&presentStoredDataCount);
		printf("Total Data Count : %d\n", presentStoredDataCount/2);
		printf("\n======================= mode ====================\n");
		printf("- s : save image start\n");
		printf("- q or ESC : quit&save\n");

		char m = _getch();

		if(m == 'q' || m == 27){
			printf("Data Recorder exit...\n");
			break;
		}
		// file 저장부
		if(m == 's'){
			char obj_name[256];
			int pose_id, tCount = 0;
			printf("Object Name : ");
			scanf("%s", obj_name);
			printf("Pose ID : ");
			scanf("%d", &pose_id);

			//디렉토리 생성부
			TCHAR szDir[MAX_PATH] = {0,};
			char dirpath[256];
			sprintf(dirpath, "%s\\%s", DEFAULT_PATH, obj_name);
			MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
			bool mkdir_check = CreateDirectory(szDir, NULL);									//루트 디렉토리
			sprintf(dirpath, "%s\\RGB\\%s", DEFAULT_PATH, obj_name);
			MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
			mkdir_check = CreateDirectory(szDir, NULL);											//컬러 디렉토리
			sprintf(dirpath, "%s\\DEPTH\\%s", DEFAULT_PATH, obj_name);
			MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
			mkdir_check = CreateDirectory(szDir, NULL);											//뎁스 디렉토리

			while(1){
				char key = cv::waitKey(OPENCV_WAIT_DELAY);

				if(key == 's')
					saveCheck *= -1;

				Kinect.GetColorImage(&KinectColorImage);
				Kinect.GetDepthImage(&KinectDepthImage);

				cv::Rect cropRect(KINECT_COLOR_WIDTH/2 - saveResolution/2,KINECT_COLOR_HEIGHT/2 - saveResolution/2,saveResolution, saveResolution);
				cv::Rect cropDepth(KINECT_DEPTH_WIDTH/2 - saveDepthResolution/2 - 20,KINECT_DEPTH_HEIGHT/2 - 240/2 + 60,saveDepthResolution, saveDepthResolution);

				//저장부
				if(FrameCount > 2 && saveCheck == 1){
					FrameCount = 0;
					cv::Mat Crop_RGB = KinectColorImage(cropRect);
					//cv::Mat Crop_Depth = KinectMappingImage(cropRect);
					cv::Mat Crop_Depth = KinectDepthImage(cropDepth);

					cv::resize(Crop_RGB, Crop_RGB, Size(240,240));
					cv::resize(Crop_Depth, Crop_Depth, Size(240,240));
					char tBuf[256];
					sprintf(tBuf, "%s\\%s\\RGB\\%d_%d.jpg", DEFAULT_PATH, obj_name, pose_id, tCount);
					imwrite(tBuf, Crop_RGB);
					sprintf(tBuf, "%s\\%s\\DEPTH\\%d_%d.jpg", DEFAULT_PATH, obj_name, pose_id, tCount);
					imwrite(tBuf, Crop_Depth);
					printf("%d_%d.jpg saved!\n", pose_id, tCount);
					tCount++;
				}

				//그리기
				Mat KinectDraw = KinectColorImage.clone();
				Mat KinectDepthDraw = KinectDepthImage.clone();
				cv::rectangle(KinectDraw, cropRect, cv::Scalar(0,255,0), 2);
				cv::rectangle(KinectDepthDraw, cropDepth, cv::Scalar(0,255,0), 2);

				imshow("KinectColorFrame", KinectDraw);
				imshow("KinectDepthFrame", KinectDepthDraw);

				FrameCount++;

				if(key == 27)
					break;
			}
			printf("%d data save complete!\n", tCount);
		}
	}

	Kinect.KinectDestroy();

	destroyAllWindows();

	return 0;
}

void printClassData(char* path, int *count){
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	TCHAR szDir[MAX_PATH] = {0,};
	int tCount = 0;

	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, path, strlen(path), szDir, MAX_PATH);
	StringCchCat(szDir, MAX_PATH, TEXT("\\*"));

	hFind = FindFirstFile(szDir, &ffd);
	while (FindNextFile(hFind, &ffd) != 0){
		if(fileTypeCheck(ffd.cFileName)){
			tCount++;
			*count += 1;
		}

		if(ffd.dwFileAttributes == 16 && ffd.cFileName[0] != '.'){

			TCHAR subDir[MAX_PATH] = {0,};
			memcpy(subDir, szDir, sizeof(TCHAR)*MAX_PATH);
			size_t len;
			StringCchLength(subDir, MAX_PATH, &len);
			subDir[len-1] = '\0';
			StringCchCat(subDir, MAX_PATH, ffd.cFileName);
			char tBuf[MAX_PATH];
			StringCchLength(subDir, MAX_PATH, &len);
			WideCharToMultiByte(CP_ACP, 0, subDir, MAX_PATH, tBuf, MAX_PATH, NULL, NULL);
			printClassData(tBuf, count);

			//printf("Class[ %ws ] : %d\n", ffd.cFileName, dirCount);
		}
	}
	if(tCount != 0){
		int tLen = strlen(DEFAULT_PATH);
		printf("%s : %d\n", &path[tLen+1], tCount);
	}
}

bool fileTypeCheck(TCHAR *fileName){
	size_t fileLen;
	StringCchLength(fileName, MAX_PATH, &fileLen);

	if(fileLen < 5)
		return false;

	if(fileName[fileLen-1] != 'g')
		return false;
	if(fileName[fileLen-2] != 'p')
		return false;
	if(fileName[fileLen-3] != 'j')
		return false;

	return true;
}

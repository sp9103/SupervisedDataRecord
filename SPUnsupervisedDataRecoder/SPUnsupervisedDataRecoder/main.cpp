#include <conio.h>
#include <strsafe.h>

#include "stdafx.h"

#include "KinectConnecter.h"
#include "EndEffectorRecord.h"
#include "RobotController\RobotArm.h"

using namespace armsdk;

void printClassData(char* path, int *count);
bool fileTypeCheck(TCHAR *fileName);
void ControllerInit(RobotArm *robot);
int WaitUntilMoveEnd(RobotArm *robot);

//각 오브젝트 클래스 & 포즈 아이디 하나당 바이너리 파일 하나가 생성됨.
//DCNN에서 읽어들일때는 바이너리 파일들을 몽조리 읽어야 이미지와 endeffector pair를 읽을수있음.

int main(){
	printf("==================Unsupervise Data recoder===================\n");
	Mat KinectColorImage;			//Kinect Color Image
	Mat KinectDepthImage;

	int saveResolution = 480*2/3;
	int saveDepthResolution = 180*2/3;
	int presentStoredDataCount = 0;
	int saveCheck = -1;
	int getEndEffector = -1;

	KinectColorImage.create(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC4);			//Kinect Color Image format BGRA 4 channel image
	KinectDepthImage.create(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_8UC4);			//Kinect Depth Image format BGRA 4 Channel image

	KinectConnecter Kinect;
	EndEffectorRecord record;
	RobotArm _controller;
	armsdk::RobotInfo robot;
	Kinematics kin;

	veci angi(6);
	armsdk::Pose3D body[6];
	armsdk::Pose3D finger[3];

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
	record.CreateRecordFile("SupervisedData.bin");

	ControllerInit(&_controller);
	WaitUntilMoveEnd(&_controller);

#ifdef RIGHT_ARM_USE
	dxl_write_dword(_controller.DXL_Get_Port(), 7, NX::P_HOMING_OFFSET_LL,  62750, 0);
#elif defined LEFT_ARM_USE
	dxl_write_dword(_controller.DXL_Get_Port(), 8, NX::P_HOMING_OFFSET_LL, -62750, 0);
#endif

	_controller.Arm_Get_JointValue(&angi);

	//맥시멈 앵글 체크 - 쓰레기값 걸러내기
	for(int JointNum = 0; JointNum < 6; JointNum++)
	{
		if(abs(angi[JointNum]) > robot.GetJointInfo(JointNum)->GetMaxAngleInValue() + 10)
		{
			cout<<"read fail"<<endl;
			return -1;
		}
	}

	_controller.TorqueOff();

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
			record.CloseCreatedFile();
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

			char binPath[256];
			sprintf(binPath, "%s\\%s_%d.bin", dirpath, obj_name, pose_id);
			record.CreateRecordFile(binPath);

			sprintf(dirpath, "%s\\%s\\RGB", DEFAULT_PATH, obj_name);
			MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
			mkdir_check = CreateDirectory(szDir, NULL);											//컬러 디렉토리
			sprintf(dirpath, "%s\\%s\\DEPTH", DEFAULT_PATH, obj_name);
			MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
			mkdir_check = CreateDirectory(szDir, NULL);											//뎁스 디렉토리

			while(1){
				char key = cv::waitKey(OPENCV_WAIT_DELAY);

				if(getEndEffector == 1){
					if(key == 's')
						saveCheck *= -1;
				}

				if(key == 'a'){
					int tempPos[9];			//6_Thumb, 7_upperLeft, 8_Upperright
					_controller.GetPresPosition(tempPos);

					int Thumbangle = tempPos[6];
					int UpperLeftangle = tempPos[7];
					int UpperRightangle = tempPos[8];
					tempPos[6] = UpperLeftangle;
					tempPos[7] = UpperRightangle;
					tempPos[8] = Thumbangle;
					kin.ForwardWithFinger(tempPos, body, finger);

					printf("Endeffector recorded.\n");

					getEndEffector = 1;
				}

				Kinect.GetColorImage(&KinectColorImage);
				Kinect.GetDepthImage(&KinectDepthImage);

				cv::Rect cropRect(KINECT_COLOR_WIDTH/2 - saveResolution/2,KINECT_COLOR_HEIGHT/2 - saveResolution/2,saveResolution, saveResolution);
				cv::Rect cropDepth(KINECT_DEPTH_WIDTH/2 - saveDepthResolution/2 - 20,KINECT_DEPTH_HEIGHT/2 - 240/2 + 60,saveDepthResolution, saveDepthResolution);

				//저장부
				if(saveCheck == 1 && getEndEffector == 1){
					cv::Mat Crop_RGB = KinectColorImage(cropRect);
					cv::Mat Crop_Depth = KinectDepthImage(cropDepth);

					cv::resize(Crop_RGB, Crop_RGB, Size(240,240));
					cv::resize(Crop_Depth, Crop_Depth, Size(240,240));
					char tBuf[256];
					sprintf(tBuf, "%s\\%s\\RGB\\%s_%d_%d.jpg", DEFAULT_PATH, obj_name, obj_name, pose_id, tCount);
					imwrite(tBuf, Crop_RGB);
					sprintf(tBuf, "%s\\%s\\DEPTH\\%s_%d_%d.jpg", DEFAULT_PATH, obj_name, obj_name, pose_id, tCount);
					imwrite(tBuf, Crop_Depth);

					saveCheck = -1;
					getEndEffector = -1;

					char RGBbuf[256], Depttbuf[256];
					sprintf(RGBbuf, "RGB\\%s_%d_%d.jpg", obj_name, pose_id, tCount);
					sprintf(Depttbuf, "DEPTH\\%s_%d_%d.jpg", obj_name, pose_id, tCount);
					record.WriteData(RGBbuf, Depttbuf, cv::Point3f(finger[0].x, finger[0].y, finger[0].z), cv::Point3f(finger[1].x, finger[1].y, finger[1].z), cv::Point3f(finger[2].x, finger[2].y, finger[2].z));

					printf("%s_%d_%d.jpg saved!\n", obj_name, pose_id, tCount);
					tCount++;
				}

				//그리기
				Mat KinectDraw = KinectColorImage.clone();
				Mat KinectDepthDraw = KinectDepthImage.clone();
				cv::rectangle(KinectDraw, cropRect, cv::Scalar(0,255,0), 2);
				cv::rectangle(KinectDepthDraw, cropDepth, cv::Scalar(0,255,0), 2);

				imshow("KinectColorFrame", KinectDraw);
				imshow("KinectDepthFrame", KinectDepthDraw);

				if(key == 27)
					break;
			}
			printf("%d data save complete!\n", tCount);

			record.CloseCreatedFile();
		}
	}

	Kinect.KinectDestroy();

	destroyAllWindows();

	_controller.DeInit();

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

void ControllerInit(RobotArm *robot){
	int robotid[] = {2,4,6,8,10,12,14,16,18};
	int vel[] = {1000, 1000, 1000, 1000, 1000, 1000, 50, 50, 50};
	int Initpos[] = {0, 0, 0, 0, 0, 0, 2440, 2400, 1700};

	robot->Init(3,3, robotid);

	robot->TorqueOff();
	robot->TorqueOn();

	robot->SetGoalVelocity(vel);
	//robot->SetGoalPosition(Initpos);
	robot->SetFingerPosition(&Initpos[6]);
}

int isAllZero(int src[]){
	for(int i = 0; i < NUM_XEL; i++){
		if(src[i] != 0)
			return -1;
	}

	return 1;
}

int WaitUntilMoveEnd(RobotArm *robot){
	int checkTerm = 10;
	int presVel[NUM_XEL];
	int fingerLoad[NUM_FINGER];

	while(1){
		_sleep(33);
		robot->GetPresVelocity(presVel);
		robot->GetFingerLoad(fingerLoad);

		if(isAllZero(presVel) == 1){

			return 1;
		}
	}

	return 1;
}
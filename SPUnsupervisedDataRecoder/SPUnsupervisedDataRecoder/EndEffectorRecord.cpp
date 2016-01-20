#include "EndEffectorRecord.h"


EndEffectorRecord::EndEffectorRecord(void)
{
	fp_ = NULL;
}


EndEffectorRecord::~EndEffectorRecord(void)
{
	if(fp_ != NULL){
		CloseOpendFile();
	}
}

//write function
void EndEffectorRecord::CreateRecordFile(char *fileName){
	if(fp_ == NULL){
		fp_ = fopen(fileName, "ab");
	}
}

void EndEffectorRecord::WriteData(char *RGBImgPath, char *DepthImgPath, cv::Point3f UpperLeft, cv::Point3f UpperRight, cv::Point3f Thumb){
	SupervisedData tempData;

	strcpy(tempData.DepthPath, DepthImgPath);
	strcpy(tempData.RGBPath, RGBImgPath);
	tempData.Thumb = Thumb;
	tempData.UpperLeft = UpperLeft;
	tempData.UpperRight = UpperRight;
}

void EndEffectorRecord::CloseCreatedFile(){
	if(fp_ != NULL){
		int dataCount = data_.size();
		printf("Write Data... Data Count : %d\n", dataCount);

		for(int i = 0; i < dataCount; i++){
			SupervisedData tempData = data_.at(i);
			int RGBPathLen = strlen(tempData.RGBPath);
			int DepthPathLen = strlen(tempData.DepthPath);

			//RGB & Depth Image Path write
			fwrite(&RGBPathLen, sizeof(int), 1, fp_);
			fwrite(tempData.RGBPath, sizeof(char), RGBPathLen, fp_);
			fwrite(&DepthPathLen, sizeof(int) ,1, fp_);
			fwrite(tempData.DepthPath, sizeof(char), DepthPathLen, fp_);

			//Endeffector write
			cv::Point3f EndTemp[3];
			EndTemp[0] = tempData.UpperLeft;
			EndTemp[1] = tempData.UpperRight;
			EndTemp[2] = tempData.Thumb;

			for(int i = 0; i < 3; i++){
				fwrite(&EndTemp[i].x, sizeof(float), 1, fp_);
				fwrite(&EndTemp[i].y, sizeof(float), 1, fp_);
				fwrite(&EndTemp[i].z, sizeof(float), 1, fp_);
			}

			printf("[%d / %d] complete!\n", i+1, dataCount);
		}

		fclose(fp_);
		fp_ = NULL;
	}
}

//Read function
void EndEffectorRecord::OpenRecordFile(char *fileName){
	if(fp_ != NULL){
		fp_ = fopen(fileName, "rb");
	}
}

int EndEffectorRecord::ReadData(cv::Mat *RGBimg, cv::Mat *DepthImg, cv::Point3f *UpperLeft, cv::Point3f *UpperRight, cv::Point3f *Thumb){
	if(feof(fp_)){
		return -1;
	}

	int RGBPathLen, DEPTHPathLen;
	char RGBPath[256], DepthPath[256];
	fread(&RGBPathLen, sizeof(int), 1, fp_);
	fread(RGBPath, sizeof(char), RGBPathLen, fp_);
	fread(&DEPTHPathLen, sizeof(int), 1, fp_);
	fread(DepthPath, sizeof(char), DEPTHPathLen, fp_);

	*RGBimg = cv::imread(RGBPath);
	*DepthImg = cv::imread(DepthPath);

	float x, y, z;
	fread(&x, sizeof(float), 1, fp_);
	fread(&y, sizeof(float), 1, fp_);
	fread(&z, sizeof(float), 1, fp_);
	*UpperLeft = cv::Point3f(x,y,z);

	fread(&x, sizeof(float), 1, fp_);
	fread(&y, sizeof(float), 1, fp_);
	fread(&z, sizeof(float), 1, fp_);
	*UpperRight = cv::Point3f(x,y,z);

	fread(&x, sizeof(float), 1, fp_);
	fread(&y, sizeof(float), 1, fp_);
	fread(&z, sizeof(float), 1, fp_);
	*Thumb = cv::Point3f(x,y,z);

	return 1;
}

void EndEffectorRecord::CloseOpendFile(){
	if(fp_ != NULL){
		fclose(fp_);
		fp_ = NULL;
	}
}
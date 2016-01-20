#ifndef __KINEMATICS_H_
#define __KINEMATICS_H_

#include <vector>
#include <iostream>
#include "RobotInfo.h"
#include "Error.h"

namespace armsdk
{
	class Kinematics
	{
	private:
		vector<JointData>* mRobotInfo;
		RobotInfo* mRobot;
		vector<matd> mRobotMat;
		Pose3D currentpose;
	//	Pose3D_Quaternion currentpose_Quaternion;
		vecd currentangle;
		unsigned int DOF;

		matd UpperLeftMat, UpperRightMat, ThumbMat;

	public:
		Kinematics();
		~Kinematics();

		void setRobotInfo(RobotInfo *_mRobot);
		
		void RobotInfoReload(void);

		matd Forward(vecd angle);
		matd Forward(vecd angle, Pose3D *pose);
		void Forward(vecd angle, float *xyz);		//angle = yaw, pitch, roll
		void EndAxis(vecd angle, Pose3D *pose, Pose3D *xaxis, Pose3D *yaxis, Pose3D *zaxis);
		void ForwardWithAxis(vecd angle, float *xyz, Pose3D *xaxis, Pose3D *yaxis, Pose3D *zaxis);
		void ForwardWithFinger(int *angle, Pose3D body[6], Pose3D finger[3]);
		//matd Forward(vecd angle, Pose3D_Quaternion *pose);
		matd Jacobian(void);
		vecd CalcError(Pose3D _desired, matd _current);
		void ComputeIK(Pose3D _desired, vecd *q, vecd Initangle, int *ErrorStatus);

		unsigned int GetNumberofJoint(void);
		vecd* GetCurrentAngle(void);
		Pose3D* GetCurrentPose(void);
		//Pose3D_Quaternion* GetCurrettPose_Quaternion(void);
		RobotInfo* GetRobotInfo(void);
		veci Rad2Value(vecd q);
		vecd Value2Rad(veci q);
		veci Get_IDList(void);

		void FingerReggression(int *angle, Pose3D xaxis, Pose3D yaxis, Pose3D zaxis, float *finger);
	};
}
#endif 
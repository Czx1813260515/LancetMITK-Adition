#pragma once
#ifndef LANCETJAKAROBOT_h
#define LANCETJAKAROBOT_h

#include "AbstractRobot.h"
#include <JAKAZuRobot.h>
#include <vtkMath.h>
#include <string>
#include "MitkLancetHardwareDeviceExports.h"
#include "PrintDataHelper.h"
//JAKA
// 定义一个宏，用于检查函数返回值是否为成功
#define CHECK_ERROR_AND_RETURN(func_call) \
    do { \
        int error_code = (func_call); \
        if (error_code != ERR_SUCC) { \
            printf("Error: %s\n", to_string(error_code)); \
            return; \
        } \
    } while(0)


class MITKLANCETHARDWAREDEVICE_EXPORT LancetJakaRobot : public AbstractRobot
{
	//Q_OBJECT
public:
	LancetJakaRobot();
	void Connect() override;

	void PowerOn() override;

	void PowerOff() override;

	void Translate(double x, double y, double z) override;

	void Translate(double* aDirection, double aLength) override;

	void Rotate(double* aDirection, double aAngle) override;

	void RecordInitialPos() override;

	void GoToInitialPos() override;

	void SetTCPToFlange() override;

	bool SetTCP(vtkMatrix4x4* aMatrix) override;

	std::vector<double> GetJointAngles() override;

	void SetJointAngles(std::vector<double> aJointAngles) override;

	vtkSmartPointer<vtkMatrix4x4> GetBaseToTCP() override;

	vtkSmartPointer<vtkMatrix4x4> GetFlangeToTCP() override;

	vtkSmartPointer<vtkMatrix4x4> GetBaseToFlange() override;

	void RobotTransformInBase(double* aMatrix) override;

	void RobotTransformInTCP(double* aMatrix) override;

	std::vector<double> GetCartStiffParams() override;

	bool SetCartStiffParams(std::vector<double> aStiff) override;

	std::vector<double> GetCartDampParams() override;

	bool SetCartDampParams(std::vector<double> aDamp) override;

	std::vector<std::vector<double>> GetJointAngleLimits() override;

	bool SetVelocity(double aVelocity) override;

	void WaitMove() override;

private:
	Eigen::Matrix3d GetRotationMatrixByEuler(double rx, double ry, double rz);
	vtkSmartPointer<vtkMatrix4x4> GetMatrixByRotationAndTranslation(Eigen::Matrix3d aRotation, Eigen::Vector3d aTranslation);
	Eigen::Vector3d GetEulerByMatrix(vtkMatrix4x4* m);
	Eigen::Vector3d CalculateXYZEulerByRotation(Eigen::Matrix3d m);
	Eigen::Matrix3d GetRotationPartByMatrix(vtkMatrix4x4* m);
	Eigen::Vector3d GetTranslationPartByMatrix(vtkMatrix4x4* m);
	std::vector<double> CalculateInverse(Eigen::Vector3d aTranslation, Eigen::Vector3d aEulerAngle);
	inline const char* to_string(int error_code)
	{
		switch (error_code)
		{
		case ERR_SUCC:                return "Success";
		case ERR_FUCTION_CALL_ERROR:  return "Function call error";
		case ERR_INVALID_HANDLER:     return "Invalid handler";
		case ERR_INVALID_PARAMETER:   return "Invalid parameter";
		case ERR_COMMUNICATION_ERR:   return "Communication error";
		case ERR_KINE_INVERSE_ERR:    return "Kinematics inverse error";
		case ERR_EMERGENCY_PRESSED:   return "Emergency stop pressed";
		case ERR_NOT_POWERED:         return "Robot not powered";
		case ERR_NOT_ENABLED:         return "Robot not enabled";
		case ERR_DISABLE_SERVOMODE:   return "Servo mode not enabled";
		case ERR_NOT_OFF_ENABLE:      return "Robot not off enable";
		case ERR_PROGRAM_IS_RUNNING:  return "Program is running, operation not allowed";
		case ERR_CANNOT_OPEN_FILE:    return "Cannot open file";
		case ERR_MOTION_ABNORMAL:     return "Motion abnormal";
		case ERR_FTP_PREFROM:         return "FTP error";
		case ERR_VALUE_OVERSIZE:      return "Socket message or value oversize";
		default:                      return "Unknown error";
		}
	}
private:
	vtkSmartPointer<vtkMatrix4x4> m_InitialPos;
	vtkSmartPointer<vtkMatrix4x4> m_FlangeToTCP;

private:
	//声明对象
	JAKAZuRobot m_Robot;
	//定义关节角速度 rad/s
	double dJointVel = 0.2;
	//定义关节加速度 rad/s^2
	double dJointAcc = 0.3;
	//定义机器人关节运动终点误差 mm
	double dJointErr = 0.01;
};

#endif
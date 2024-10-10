#pragma once
#ifndef LANCETROBOTREGISTRATION_h
#define LANCETROBOTREGISTRATION_h
#include "MitkLancetRobotRegistrationExports.h"

#include "AbstractCamera.h"
#include "AbstractRobot.h"
#include "AimCamera.h"
#include "LancetHansRobot.h"
#include "robotRegistration.h"
#include "qdatetime.h"
class MITKLANCETROBOTREGISTRATION_EXPORT LancetRobotRegistration
{
public:
	LancetRobotRegistration(AbstractRobot* aRobot, AbstractCamera* aCamera);
	void setTCPToFlange();
	void recordInitialPos();
	void goToInitialPos();
	void xp();
	void yp();
	void zp();
	void xm();
	void ym();
	void zm();
	void rxp();
	void ryp();
	void rzp();
	void rxm();
	void rym();
	void rzm();
	int captureRobot();
	void capturePose(bool);
	void waitMove();
	void autoCollection();
	void setDistance(int);
	void setAngle(int);
public:
	
private:
	AbstractRobot* m_Robot;
	AbstractCamera* m_Camera;
	RobotRegistration m_RobotRegistration;
	bool isAutoCollectionFlag;
	int Distance = 50;
	int Angle = 15;
private:
	void Sleep(int);
};
#endif
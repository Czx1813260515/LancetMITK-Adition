#pragma once
#include <qwidget.h>
#include <qobject.h>
#include "ui_HansRobotControls.h"
#include "LancetHansRobot.h"
#include "FileIO.h"
#include "AimCamera.h"

#include <QmitkSingleNodeSelectionWidget.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateProperty.h>
#include <mitkNodePredicateCompositeBase.h>
#include <mitkSurface.h>
#include <mitkNodePredicateBase.h>
#include <mitkNodePredicateDataType.h>
#include <mitkNodePredicateOr.h>
#include <mitkColorProperty.h>

class RobotArmRegistrationTab : public QWidget
{

	Q_OBJECT
public:
	RobotArmRegistrationTab(Ui::HansRobotControls ui, mitk::DataStorage* aDataStorage, AbstractRobot* aRobot, AbstractCamera* aCamera, QWidget* parent = nullptr);

public slots:
	

	//void flashLable(Ui::HansRobotControls* lable);
private:
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

private:
	QWidget* m_TabPage;
	Ui::HansRobotControls m_ui;
	//AbstractRobot* m_Robot;
	LancetHansRobot* m_Robot;
	mitk::DataStorage* m_dataStorage;
	AbstractCamera* m_Camera;

	void flashLable(Eigen::Vector3d tempTip, QLabel* label);
};
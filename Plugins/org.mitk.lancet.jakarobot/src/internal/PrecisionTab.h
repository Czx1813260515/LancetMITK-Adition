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

class PrecisionTab :  public QWidget
{
	Q_OBJECT
public:
	explicit PrecisionTab(Ui::HansRobotControls ui, mitk::DataStorage* aDataStorage, LancetHansRobot* HansRobotHardwareService,AimCamera* AimCameraHardwareService, QWidget* parent = nullptr);

public slots:
	void InitConnection();


private:
	

private:
	QWidget* m_TabPage;
	Ui::HansRobotControls m_ui;
	//lancetAlgorithm::SystemPrecision* m_SystemPrecision;
	LancetHansRobot* m_HansRobotHardwareService;
	mitk::DataStorage* m_dataStorage;

};
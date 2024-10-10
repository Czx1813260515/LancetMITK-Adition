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

class RoboArmRegistrationTab //: public QWidget
{
	//Q_OBJECT
public:
	 RoboArmRegistrationTab()
	{

	}
};
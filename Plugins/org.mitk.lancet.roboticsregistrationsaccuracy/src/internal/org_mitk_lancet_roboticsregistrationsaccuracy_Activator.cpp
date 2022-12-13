/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#include "org_mitk_lancet_roboticsregistrationsaccuracy_Activator.h"
#include "QRoboticsRegistrationsAccuracy.h"

namespace mitk
{
	static ctkPluginContext* staticPluginActivator = nullptr;

  void org_mitk_lancet_roboticsregistrationsaccuracy_Activator::start(ctkPluginContext *context)
  {
		staticPluginActivator = context;
    BERRY_REGISTER_EXTENSION_CLASS(QRoboticsRegistrationsAccuracy, context)
  }

  void org_mitk_lancet_roboticsregistrationsaccuracy_Activator::stop(ctkPluginContext*) 
	{
		staticPluginActivator = nullptr;
	}

	ctkPluginContext* org_mitk_lancet_roboticsregistrationsaccuracy_Activator::GetPluginContext()
	{
		return staticPluginActivator;
	}
}

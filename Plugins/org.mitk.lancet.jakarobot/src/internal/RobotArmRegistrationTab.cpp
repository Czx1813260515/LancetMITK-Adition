#include "..\..\..\org.mitk.lancet.hansrobot\src\internal\RobotArmRegistrationTab.h"
#include "RobotArmRegistrationTab.h"/// <summary>

/// Robots the arm registration tab.
/// </summary>
/// <param name="ui">The UI.</param>
/// <param name="aDataStorage">a data storage.</param>
/// <param name="aRobot">a robot.</param>
/// <param name="aCamera">a camera.</param>
/// <param name="parent">The parent.</param>
/// <returns></returns>
RobotArmRegistrationTab::RobotArmRegistrationTab(Ui::HansRobotControls ui, mitk::DataStorage* aDataStorage, AbstractRobot* aRobot, AbstractCamera* aCamera, QWidget* parent)
{
	m_ui = ui;
	m_Robot = dynamic_cast<LancetHansRobot*>(aRobot);
	m_Camera = aCamera;
	m_dataStorage = aDataStorage;
	if (!m_dataStorage)
	{
		std::cout << "m_dataStorage is nullptr" << std::endl;
	}
	//�ƶ�
	connect(m_ui.pushButton_xp_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::xp);
	connect(m_ui.pushButton_yp_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::yp);
	connect(m_ui.pushButton_zp_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::zp);
	connect(m_ui.pushButton_xm_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::xm);
	connect(m_ui.pushButton_ym_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::ym);
	connect(m_ui.pushButton_zm_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::zm);
	connect(m_ui.pushButton_rxp_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::rxp);
	connect(m_ui.pushButton_ryp_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::ryp);
	connect(m_ui.pushButton_rzp_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::rzp);
	connect(m_ui.pushButton_rxm_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::rxm);
	connect(m_ui.pushButton_rym_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::rym);
	connect(m_ui.pushButton_rzm_2, &QPushButton::clicked, this, &RobotArmRegistrationTab::rzm);
	//��׼
	connect(m_ui.pushButton_setTCPToFlange_2, &QPushButton::clicked, this, [this]() {
			m_Robot->SetTCPToFlange();
			m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_setInitialPoint_2, &QPushButton::clicked, this, [this]() {
			m_Robot->RecordInitialPos();
			m_ui.textBrowser_hans->append(QString("RecordInitialPos"));
		});
	connect(m_ui.pushButton_goToInitial_2, &QPushButton::clicked, this, [this]() {
			m_Robot->GoToInitialPos();
			m_ui.textBrowser_hans->append(QString("GoToInitialPos"));
		});
	//���Ȳ���
	connect(m_ui.pushButton_repeat_position, &QPushButton::clicked, this, &RobotArmRegistrationTab::RepeatPositionTest);
	connect(m_ui.pushButton_aboslute_position, &QPushButton::clicked, this, &RobotArmRegistrationTab::AboslutePositionTest);
}

void RobotArmRegistrationTab::xp()
{
	int intValue = m_ui.lineEdit_intuitiveValue_2->text().toInt();
	double Distance = static_cast<double>(intValue);
	double templeArray[3] = { 1, 0.0, 0.0 };
	m_Robot->Translate(templeArray, Distance);
	m_ui.textBrowser_hans->append(QString("xp"));

}
void RobotArmRegistrationTab::yp()
{
	int intValue = m_ui.lineEdit_intuitiveValue_2->text().toInt();
	double Distance = static_cast<double>(intValue);
	double templeArray[3] = { 0.0, 1, 0.0 };
	m_Robot->Translate(templeArray, Distance);
	m_ui.textBrowser_hans->append(QString("yp"));

}
void RobotArmRegistrationTab::zp()
{
	int intValue = m_ui.lineEdit_intuitiveValue_2->text().toInt();
	double Distance = static_cast<double>(intValue);
	double templeArray[3] = { 0.0, 0.0, 1 };
	m_Robot->Translate(templeArray, Distance);

}
void RobotArmRegistrationTab::xm()
{
	 int intValue = m_ui.lineEdit_intuitiveValue_2->text().toInt();
	 double Distance = -static_cast<double>(intValue);
	 double templeArray[3] = { 1, 0.0, 0.0 };
	 m_Robot->Translate(templeArray, Distance);


}

void RobotArmRegistrationTab::ym()
{

	 int intValue = m_ui.lineEdit_intuitiveValue_2->text().toInt();
	 double Distance = -static_cast<double>(intValue);
	 double templeArray[3] = { 0.0, 1, 0.0 };
	 m_Robot->Translate(templeArray, Distance);

}
void RobotArmRegistrationTab::zm()
{
	 int intValue = m_ui.lineEdit_intuitiveValue_2->text().toInt();
	 double Distance = -static_cast<double>(intValue);
	 double templeArray[3] = { 0.0, 0.0, 1 };
	 m_Robot->Translate(templeArray, Distance);


}
void RobotArmRegistrationTab::rxp()
{
	int intValue = m_ui.lineEdit_intuitiveValue_degree_2->text().toInt();
	double Angle = static_cast<double>(intValue);
	double templeArray[3] = { 1, 0.0, 0.0 };
	m_Robot->Rotate(templeArray, Angle);

}
void RobotArmRegistrationTab::ryp()
{
	int intValue = m_ui.lineEdit_intuitiveValue_degree_2->text().toInt();
	double Angle = static_cast<double>(intValue);
	double templeArray[3] = { 0.0, 1, 0.0 };
	m_Robot->Rotate(templeArray, Angle);

}
void RobotArmRegistrationTab::rzp()
{
	 int intValue = m_ui.lineEdit_intuitiveValue_degree_2->text().toInt();
	 double Angle = static_cast<double>(intValue);
	 double templeArray[3] = { 0.0, 0.0, 1 };
	 m_Robot->Rotate(templeArray, Angle);

}
void RobotArmRegistrationTab::rxm()
{
	 int intValue = m_ui.lineEdit_intuitiveValue_degree_2->text().toInt();
	 double Angle = -static_cast<double>(intValue);
	 double templeArray[3] = { 1, 0.0, 0.0 };
	 m_Robot->Rotate(templeArray, Angle);


}
void RobotArmRegistrationTab::rym()
{
	 int intValue = m_ui.lineEdit_intuitiveValue_degree_2->text().toInt();
	 double Angle = -static_cast<double>(intValue);
	 double templeArray[3] = { 0.0, 1, 0.0 };
	 m_Robot->Rotate(templeArray, Angle);



}
void RobotArmRegistrationTab::rzm()
{
	int intValue = m_ui.lineEdit_intuitiveValue_degree_2->text().toInt();
	double Angle = -static_cast<double>(intValue);
	double templeArray[3] = { 0.0, 0.0, 1 };
	m_Robot->Rotate(templeArray, Angle);

	
}

void RobotArmRegistrationTab::captureRobot()
{
	m_ui.textBrowser_hans->append(QString("captureRobot"));

	if (m_RobotRegistration.PoseCount() < 5) //The first five translations, 
	{
		CapturePose(true);
	}
	else if (m_RobotRegistration.PoseCount() >= 5 && m_RobotRegistration.PoseCount() < 10)
	{
		CapturePose(false);
	}
	else
	{
		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
		m_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);

		robotEndToFlangeMatrix->Invert();
		m_ui.textBrowser_hans->append("Registration RMS: " + QString::number(m_RobotRegistration.RMS()));
		std::cout << "Registration RMS: " << m_RobotRegistration.RMS() << std::endl;
	}

	m_ui.lineEdit_collectedRoboPose_2->setText(QString::number(m_RobotRegistration.PoseCount()));

}

void RobotArmRegistrationTab::CapturePose(bool translationOnly)
{
	m_ui.textBrowser_hans->append(QString("CapturePose"));
	//get T_BaseToFlanger
	vtkSmartPointer<vtkMatrix4x4>T_BaseToFlange= m_Robot->GetBaseToFlange();
	//get T_CameraToEnd
	vtkSmartPointer<vtkMatrix4x4>T_CameraToEnd= m_Camera->GetToolMatrixByName("RobotEndRF");
	//get T_BaseRFToCamera
	vtkSmartPointer<vtkMatrix4x4>T_BaseRFToCamera;
	vtkMatrix4x4::Invert(m_Camera->GetToolMatrixByName("RobotBaseRF"),T_BaseRFToCamera);
	//get T_BaseRFToEnd
	vtkSmartPointer<vtkMatrix4x4> T_BaseRFToEnd;
	
	vtkMatrix4x4::Multiply4x4(T_BaseRFToCamera, T_CameraToEnd, T_BaseRFToEnd);
	//Robotic arm registration
	m_RobotRegistration.AddPoseWithVtkMatrix(T_BaseToFlange, T_BaseRFToEnd, translationOnly);
}

void RobotArmRegistrationTab::waitMove()
{
	m_ui.textBrowser_hans->append(QString("waitMove"));
	bool bDone = false;
	while (!bDone)
	{
		int nRet = HRIF_IsMotionDone(0, 0, bDone);
		if (bDone == false)
		{
			QThread::msleep(10);
			QApplication::processEvents();
			std::cout << "is still moving" << std::endl;
		}
		else
		{
			int ret = HRIF_GrpStop(0, 0);
			m_ui.textBrowser_hans->append(QString("robot is stop please continue"));
			return;
		}
	}
}

void RobotArmRegistrationTab::autoCollection()
{
	m_ui.textBrowser_hans->append(QString("autoCollection"));
	
	if (HRIF_IsConnected(0))
	{
		m_ui.textBrowser_hans->append("Starting automatic registration!");
		int moveCount = 1;
		m_RobotRegistration.RemoveAllPose();
		if (isAutoCollectionFlag)
		{
			m_Robot->SetTCPToFlange();
			isAutoCollectionFlag = FALSE;
		}
		while (moveCount <= 10)
		{
			m_Robot->GoToInitialPos();
			waitMove();
			switch (moveCount) {
			case 1:
				xp();
				waitMove();
				QThread::msleep(200);
				
			
				captureRobot();
				break;
			case 2:
				yp();
				waitMove();
				QThread::msleep(200);

				captureRobot();
				break;
			case 3:
				zp();
				waitMove();
				QThread::msleep(200);
	
				captureRobot();
				break;
			case 4:
				xm();
				waitMove();
				QThread::msleep(200);
	
				captureRobot();
				break;
			case 5:
				ym();
				waitMove();
				QThread::msleep(200);
			
				captureRobot();
				break;
			case 6:
				rxp();
				waitMove();
				QThread::msleep(200);
		
				captureRobot();
				break;
			case 7:
				ryp();
				waitMove();
				QThread::msleep(200);
		
				captureRobot();
				break;
			case 8:
				rzp();
				waitMove();
				QThread::msleep(200);
	
				captureRobot();
				break;
			case 9:
				rxm();
				waitMove();
				QThread::msleep(200);
	
				captureRobot();
				break;
			case 10:
				rym();
				waitMove();
				QThread::msleep(200);
	
				captureRobot();
				break;
			default:
				return;
			}
			QThread::msleep(200);
			moveCount++;
		}
		if (m_RobotRegistration.PoseCount() == 10)
		{
			m_ui.textBrowser_hans->append("Fully automatic registration successed! ");
		}
	}
	else
	{
		m_ui.textBrowser_hans->append("Fully automatic registration failed! ");
		m_ui.textBrowser_hans->append("RobotArm disconnect, please connect it!");
	}

}


void RobotArmRegistrationTab::RepeatPositionTest()
{
	m_Robot->RecordInitialPos();
	m_Robot->Translate(50,0,0);//xyz
	m_Robot->GoToInitialPos();
}

void RobotArmRegistrationTab::AboslutePositionTest()
{
	double cube_length = 300.0; // ������߳�����λ��mm

	// �� X ���ƶ�
	m_Robot->Translate(cube_length, 0, 0); // �� (0, 0, 0) �ƶ��� (300, 0, 0)

	// �� Y ���ƶ�
	m_Robot->Translate(0, cube_length, 0); // �� (300, 0, 0) �ƶ��� (300, 300, 0)

	// �� Z ���ƶ�
	m_Robot->Translate(0, 0, cube_length); // �� (300, 300, 0) �ƶ��� (300, 300, 300)

	// �� -X ���ƶ�
	m_Robot->Translate(-cube_length, 0, 0); // �� (300, 300, 300) �ƶ��� (0, 300, 300)

	// �� -Y ���ƶ�
	m_Robot->Translate(0, -cube_length, 0); // �� (0, 300, 300) �ƶ��� (0, 0, 300)

	// �� -Z ���ƶ�
	m_Robot->Translate(0, 0, -cube_length); // �� (0, 0, 300) �ƶ��� (0, 0, 0)
}







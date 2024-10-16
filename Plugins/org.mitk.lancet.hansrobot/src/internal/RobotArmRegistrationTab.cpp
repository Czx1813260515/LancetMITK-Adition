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
	m_Robot = dynamic_cast<DianaRobot*>(aRobot);
	m_Camera = aCamera;
	m_dataStorage = aDataStorage;
	if (!m_dataStorage)
	{
		std::cout << "m_dataStorage is nullptr" << std::endl;
	}
	//�ƶ�
	m_LancetRobotRegistration = new LancetRobotRegistration(aRobot, aCamera);

	connect(m_ui.pushButton_xp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->xp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_yp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->yp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_zp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->zp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_xm_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->xm();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_ym_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->ym();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_zm_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->zm();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rxp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rxp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_ryp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->ryp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rzp_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rzp();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rxm_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rxm();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rym_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rym();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_rzm_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->rzm();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	//��׼
	connect(m_ui.pushButton_setTCPToFlange_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->setTCPToFlange();
		m_ui.textBrowser_hans->append(QString("SetTCPToFlange"));
		});
	connect(m_ui.pushButton_setInitialPoint_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->recordInitialPos();
		m_ui.textBrowser_hans->append(QString("RecordInitialPos"));
		});
	connect(m_ui.pushButton_goToInitial_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->goToInitialPos();
		m_ui.textBrowser_hans->append(QString("GoToInitialPos"));
		});
	connect(m_ui.pushButton_captureRobot_2, &QPushButton::clicked, this, [this]() {
		m_ui.textBrowser_hans->append(QString("captureRobot"));
		m_LancetRobotRegistration->captureRobot();
		});
	connect(m_ui.pushButton_replaceRegistration_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->replaceRegistration();
		m_ui.textBrowser_hans->append(QString("Replace Registration"));
		m_ui.lineEdit_collectedRoboPose_2->setText(QString::number(0));
		});
	connect(m_ui.pushButton_saveArmMatrix_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->saveArmMatrix();
		m_ui.textBrowser_hans->append(QString("Save ArmMatrix"));
		});
	connect(m_ui.pushButton_reuseArmMatrix_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->reuseArmMatrix();
		m_ui.textBrowser_hans->append(QString("Reuse ArmMatrix"));
		m_ui.lineEdit_collectedRoboPose_2->setText(QString::number(10));
		});
	connect(m_ui.pushButton_AutoMoveJ_2, &QPushButton::clicked, this, [this]() {
		m_LancetRobotRegistration->autoCollection();
		m_ui.textBrowser_hans->append(QString("Auto Collection"));
		});
	connect(m_LancetRobotRegistration, &LancetRobotRegistration::countPose,
		this, &RobotArmRegistrationTab::updateUiCaputreCount);

	connect(m_ui.pushButton_RepeatPositionTest, &QPushButton::clicked, this, &RobotArmRegistrationTab::RepeatPositionTest);
	connect(m_ui.pushButton_AboslutePositionTest, &QPushButton::clicked, this, &RobotArmRegistrationTab::AboslutePositionTest);
}

void RobotArmRegistrationTab::updateUiCaputreCount(int cnt)
{ 
	m_ui.lineEdit_collectedRoboPose_2->setText(QString::number(cnt));
}

void RobotArmRegistrationTab::RepeatPositionTest()
{
	m_Robot->RecordInitialPos();
	m_Robot->Translate(50, 0, 0);//xyz
	m_Robot->GoToInitialPos();
}

void RobotArmRegistrationTab::AboslutePositionTest()
{
	// ������ı߳�����λ��mm
	double cube_length = 100;

	// ����������
	const char* pointNames[] = { "a", "b", "c", "d", "e", "f", "g", "h" };

	// ѡ��Ҫ�ƶ����ĵ�
	switch (countNum)
	{
	case 0:
		m_Robot->Translate(cube_length, 0, 0); // ��ʼ��
		std::cout << "�ƶ�����ʼ��: " << pointNames[0] << std::endl; // ��ӡ������
		break;
	case 1:
		m_Robot->Translate(0, cube_length, 0); // �ƶ������� 1
		std::cout << "�ƶ�����: " << pointNames[1] << std::endl; // ��ӡ������
		break;
	case 2:
		m_Robot->Translate(-cube_length, 0, 0); // �ƶ������� 2
		std::cout << "�ƶ�����: " << pointNames[2] << std::endl; // ��ӡ������
		break;
	case 3:
		m_Robot->Translate(0, 0, cube_length); // �ƶ������� 3
		std::cout << "�ƶ�����: " << pointNames[3] << std::endl; // ��ӡ������
		break;
	case 4:
		m_Robot->Translate(0, -cube_length, 0); // �ƶ������� 4
		std::cout << "�ƶ�����: " << pointNames[4] << std::endl; // ��ӡ������
		break;
	case 5:
		m_Robot->Translate(cube_length, 0, 0); // �ƶ������� 5
		std::cout << "�ƶ�����: " << pointNames[5] << std::endl; // ��ӡ������
		break;
	case 6:
		m_Robot->Translate(0, cube_length, 0); // �ƶ������� 6
		std::cout << "�ƶ�����: " << pointNames[6] << std::endl; // ��ӡ������
		break;
	case 7:
		m_Robot->Translate(-cube_length, 0, 0); // �ƶ������� 7
		std::cout << "�ƶ�����: " << pointNames[7] << std::endl; // ��ӡ������
		break;
	default:
		// ��� countNum ���� 7������Ϊ 0
		countNum = 0; 
		return; 
	}

	countNum++; // ���� countNum��׼����һ���ƶ�
}






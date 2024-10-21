#pragma once
#include <qwidget.h>
#include <qobject.h>
#include <ui_InstantiationCameraControls.h>

#include <mitkDataStorage.h>
#include <AriemediCamera.h>
#include "CameraRectLabel.h"

class CameraConnectionTab : public QWidget
{
	Q_OBJECT
public:
	explicit CameraConnectionTab(Ui::InstantiationCameraControls ui, mitk::DataStorage* aDataStorage,
		AriemediCamera* aAriemediCamera, QWidget* parent = nullptr);
	void InitConnection();
	void CameraConnectBtnClicked();

	void CameraDisconnectBtnClicked();

	void CameraStartBtnClicked();

	void DrawRectBtnClicked();

private slots:
	void UpdateUIDisplay();

private:
	void UpdateUIToolsData();
	void UpdateUIImages();
	void InitUI();
	void GetAndUpdateToolTip(Eigen::Vector3d tempTip, QLabel* label);
	void UpdateSingleImage(char* aImage, QLabel* aLabel);

private:
	Ui::InstantiationCameraControls m_UI;
	mitk::DataStorage* m_DataStorage;
	AriemediCamera* m_Camera;
	
	double m_ImageWidth;
	double m_ImageHeight;
	CameraRectLabel*  m_CameraRectLabel;
};


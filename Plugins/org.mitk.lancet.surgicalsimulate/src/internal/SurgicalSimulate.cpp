/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "SurgicalSimulate.h"

// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>
#include <mitkAffineTransform3D.h>
#include <mitkMatrixConvert.h>

//igt
#include <lancetVegaTrackingDevice.h>
#include <kukaRobotDevice.h>
#include <lancetApplyDeviceRegistratioinFilter.h>
#include <mitkNavigationDataToPointSetFilter.h>
#include <lancetPathPoint.h>

#include "lancetTrackingDeviceSourceConfigurator.h"
#include "mitkNavigationToolStorageDeserializer.h"
#include <QtWidgets\qfiledialog.h>

const std::string SurgicalSimulate::VIEW_ID = "org.mitk.views.surgicalsimulate";

void SurgicalSimulate::SetFocus()
{
  m_Controls.pushButton_connectKuka->setFocus();
}


void SurgicalSimulate::OnVirtualDevice2VisualizeTimer()
{
  //Here we call the Update() method from the Visualization Filter. Internally the filter checks if
  //new NavigationData is available. If we have a new NavigationData the cone position and orientation
  //will be adapted.
  if (m_VirtualDevice2Visualizer.IsNotNull())
  {
    m_VirtualDevice2Visualizer->Update();
    auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
    mitk::RenderingManager::GetInstance()->InitializeViews(geo);
    this->RequestRenderWindowUpdate();
  }
}

void SurgicalSimulate::CreateQtPartControl(QWidget* parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);
  InitSurfaceSelector(m_Controls.mitkNodeSelectWidget_metaImageNode);
  InitSurfaceSelector(m_Controls.mitkNodeSelectWidget_surface_regis);
  InitPointSetSelector(m_Controls.mitkNodeSelectWidget_landmark_src);
  connect(m_Controls.pushButton_connectKuka, &QPushButton::clicked, this, &SurgicalSimulate::UseKuka);
  //connect(m_Controls.pushButton_connectKuka, &QPushButton::clicked, this, &SurgicalSimulate::UseVirtualDevice2);
  connect(m_Controls.pushButton_connectVega, &QPushButton::clicked, this, &SurgicalSimulate::UseVega);
  //connect(m_Controls.pushButton_connectVega, &QPushButton::clicked, this, &SurgicalSimulate::UseVirtualDevice1);
  connect(m_Controls.pushButton_captureRobot, &QPushButton::clicked, this, &SurgicalSimulate::OnRobotCapture);
  connect(m_Controls.pushButton_automove, &QPushButton::clicked, this, &SurgicalSimulate::OnAutoMove);
  connect(m_Controls.pushButton_selfcheck, &QPushButton::clicked, this, &SurgicalSimulate::OnSelfCheck);
  connect(m_Controls.pushButton_resetRobotReg, &QPushButton::clicked, this, &SurgicalSimulate::OnResetRobotRegistration);
  connect(m_Controls.pushButton_assembleNavigationObject, &QPushButton::clicked, this, &SurgicalSimulate::SetupNavigatedImage);
  connect(m_Controls.pushButton_collectLandmark, &QPushButton::clicked, this, &SurgicalSimulate::CollectLanmarkProbe);
  connect(m_Controls.pushButton_applyRegistration, &QPushButton::clicked, this, &SurgicalSimulate::ApplySurfaceRegistration);
  connect(m_Controls.pushButton_collectIcp, &QPushButton::clicked, this, &SurgicalSimulate::CollectIcpProbe);
  connect(m_Controls.pushButton_startTracking, &QPushButton::clicked, this, &SurgicalSimulate::StartTracking);
  connect(m_Controls.pushButton_captureSurgicalPlane, &QPushButton::clicked, this,
          &SurgicalSimulate::OnCaptureProbeAsSurgicalPlane);
  connect(m_Controls.pushButton_startAutoPosition, &QPushButton::clicked, this, &SurgicalSimulate::OnAutoPositionStart);
}

void SurgicalSimulate::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
                                          const QList<mitk::DataNode::Pointer>& nodes)
{
  // iterate all selected objects, adjust warning visibility
  // foreach (mitk::DataNode::Pointer node, nodes)
  // {
  //   if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
  //   {
  //     m_Controls.labelWarning->setVisible(false);
  //     m_Controls.buttonPerformImageProcessing->setEnabled(true);
  //     return;
  //   }
  // }
  //
  // m_Controls.labelWarning->setVisible(true);
  // m_Controls.buttonPerformImageProcessing->setEnabled(false);
}

void SurgicalSimulate::UseVega()
{
  //read in filename
  QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Tool Storage"), "/",
                                                  tr("Tool Storage Files (*.IGTToolStorage)"));
  if (filename.isNull()) return;

  //read tool storage from disk
  std::string errorMessage = "";
  mitk::NavigationToolStorageDeserializer::Pointer myDeserializer = mitk::NavigationToolStorageDeserializer::New(
    GetDataStorage());
  m_VegaToolStorage = myDeserializer->Deserialize(filename.toStdString());
  m_VegaToolStorage->SetName(filename.toStdString());

  //! [UseKuka 1]
  //Here we want to use the Kuka robot as a tracking device. Therefore we instantiate a object of the class
  //KukaRobotDevice and make some settings which are necessary for a proper connection to the device.
  MITK_INFO << "Vega tracking";
  //QMessageBox::warning(nullptr, "Warning", "You have to set the parameters for the NDITracking device inside the code (QmitkIGTTutorialView::OnStartIGT()) before you can use it.");
  lancet::NDIVegaTrackingDevice::Pointer vegaTrackingDevice = lancet::NDIVegaTrackingDevice::New(); //instantiate

  //Create Navigation Data Source with the factory class, and the visualize filter.
  lancet::TrackingDeviceSourceConfiguratorLancet::Pointer kukaSourceFactory =
    lancet::TrackingDeviceSourceConfiguratorLancet::New(m_VegaToolStorage, vegaTrackingDevice);

  m_VegaSource = kukaSourceFactory->CreateTrackingDeviceSource(m_VegaVisualizer);

  m_VegaSource->Connect();

  m_VegaSource->StartTracking();

  //update visualize filter by timer
  if (m_VegaVisualizeTimer == nullptr)
  {
    m_VegaVisualizeTimer = new QTimer(this); //create a new timer
  }
  connect(m_VegaVisualizeTimer, SIGNAL(timeout()), this, SLOT(OnVegaVisualizeTimer()));
  //connect the timer to the method OnTimer()
  connect(m_VegaVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateToolStatusWidget()));
  //connect the timer to the method OnTimer()
  ShowToolStatus_Vega();
  m_VegaVisualizeTimer->start(100); //Every 100ms the method OnTimer() is called. -> 10fps

  auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
  mitk::RenderingManager::GetInstance()->InitializeViews(geo);
}

void SurgicalSimulate::GeneratePoses()
{
  //Use current pose as reference to generate new poses
  mitk::NavigationData::Pointer nd_robot2flange = m_KukaSource->GetOutput(0)->Clone();

  vtkMatrix4x4* vtkMatrix = vtkMatrix4x4::New();
  mitk::TransferItkTransformToVtkMatrix(nd_robot2flange->GetAffineTransform3D().GetPointer(), vtkMatrix);


  //
}

void SurgicalSimulate::CapturePose(bool translationOnly)
{
  //Output sequence is the same as AddTool sequence
  //get navigation data of flange in robot coords,
  mitk::NavigationData::Pointer nd_robot2flange = m_KukaSource->GetOutput(0);

  //get navigation data of RobotEndRF in ndi coords,
  //auto RobotEndRF = m_VegaToolStorage->GetToolIndexByName("RobotEndRF");
  mitk::NavigationData::Pointer nd_Ndi2RobotEndRF = m_VegaSource->GetOutput("RobotEndRF");
  //get navigation data of RobotBaseRF in ndi coords,
  //auto RobotBaseRF = m_VegaToolStorage->GetToolIndexByName("RobotBaseRF");
  mitk::NavigationData::Pointer nd_Ndi2RobotBaseRF = m_VegaSource->GetOutput("RobotBaseRF");
  //get navigation data RobotEndRF in reference frame RobotBaseRF
  mitk::NavigationData::Pointer nd_RobotBaseRF2RobotEndRF = GetNavigationDataInRef(
    nd_Ndi2RobotEndRF, nd_Ndi2RobotBaseRF);

  //add nd to registration module
  m_RobotRegistration.AddPose(nd_robot2flange, nd_RobotBaseRF2RobotEndRF, translationOnly);

  MITK_INFO << nd_robot2flange;
  MITK_INFO << nd_RobotBaseRF2RobotEndRF;
}

mitk::NavigationData::Pointer SurgicalSimulate::GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
                                                                       mitk::NavigationData::Pointer nd_ref)
{
  mitk::NavigationData::Pointer res = mitk::NavigationData::New();
  res->Graft(nd);
  res->Compose(nd_ref->GetInverse());
  return res;
}

SurgicalSimulate::~SurgicalSimulate()
{
  if (m_KukaVisualizeTimer != nullptr)
  {
    m_KukaVisualizeTimer->stop();
  }
  if (m_VegaVisualizeTimer != nullptr)
  {
    m_VegaVisualizeTimer->stop();
  }
}

void SurgicalSimulate::UseKuka()
{
  //read in filename
  QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Tool Storage"), "/",
                                                  tr("Tool Storage Files (*.IGTToolStorage)"));
  if (filename.isNull()) return;

  //read tool storage from disk
  std::string errorMessage = "";
  mitk::NavigationToolStorageDeserializer::Pointer myDeserializer = mitk::NavigationToolStorageDeserializer::New(
    GetDataStorage());
  m_KukaToolStorage = myDeserializer->Deserialize(filename.toStdString());
  m_KukaToolStorage->SetName(filename.toStdString());

  //! [UseKuka 1]
  //Here we want to use the Kuka robot as a tracking device. Therefore we instantiate a object of the class
  //KukaRobotDevice and make some settings which are necessary for a proper connection to the device.
  MITK_INFO << "Kuka tracking";
  //QMessageBox::warning(nullptr, "Warning", "You have to set the parameters for the NDITracking device inside the code (QmitkIGTTutorialView::OnStartIGT()) before you can use it.");
  m_KukaTrackingDevice = lancet::KukaRobotDevice::New(); //instantiate

  //Create Navigation Data Source with the factory class, and the visualize filter.
  lancet::TrackingDeviceSourceConfiguratorLancet::Pointer kukaSourceFactory =
    lancet::TrackingDeviceSourceConfiguratorLancet::New(m_KukaToolStorage, m_KukaTrackingDevice);

  m_KukaSource = kukaSourceFactory->CreateTrackingDeviceSource(m_KukaVisualizer);
  // m_KukaSource->RegisterAsMicroservice(); 

  m_KukaSource->Connect();
}

void SurgicalSimulate::StartTracking()
{
  if (m_KukaTrackingDevice->GetState() == 1) //ready
  {
    m_KukaSource->StartTracking();

    //update visualize filter by timer
    if (m_KukaVisualizeTimer == nullptr)
    {
      m_KukaVisualizeTimer = new QTimer(this); //create a new timer
    }
    connect(m_KukaVisualizeTimer, SIGNAL(timeout()), this, SLOT(OnKukaVisualizeTimer()));
    //connect the timer to the method OnTimer()
    connect(m_KukaVisualizeTimer, SIGNAL(timeout()), this, SLOT(UpdateToolStatusWidget()));
    //connect the timer to the method OnTimer()
    ShowToolStatus_Kuka();
    m_KukaVisualizeTimer->start(100); //Every 100ms the method OnTimer() is called. -> 10fps
  }
  else
  {
    MITK_ERROR << "Tracking not start,Device State:" << m_KukaTrackingDevice->GetState();
  }
  auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
  mitk::RenderingManager::GetInstance()->InitializeViews(geo);
}

void SurgicalSimulate::OnKukaVisualizeTimer()
{
  //Here we call the Update() method from the Visualization Filter. Internally the filter checks if
  //new NavigationData is available. If we have a new NavigationData the cone position and orientation
  //will be adapted.
  if (m_KukaVisualizer.IsNotNull())
  {
    m_KukaVisualizer->Update(); //todo Crash When close plugin
    this->RequestRenderWindowUpdate();
  }
}

void SurgicalSimulate::OnSelfCheck()
{
  // if (m_KukaTrackingDevice.IsNotNull() && m_KukaTrackingDevice->GetState()!=0)
  // {
  m_KukaTrackingDevice->RequestExecOperate(/*"Robot",*/ "setio", {"20", "20"});
  // }
  // else
  // {
  //   MITK_ERROR << "robot not connect";
  // }
}

void SurgicalSimulate::OnVegaVisualizeTimer()
{
  //Here we call the Update() method from the Visualization Filter. Internally the filter checks if
  //new NavigationData is available. If we have a new NavigationData the cone position and orientation
  //will be adapted.
  if (m_VegaVisualizer.IsNotNull())
  {
    m_VegaVisualizer->Update();
    // auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
    // mitk::RenderingManager::GetInstance()->InitializeViews(geo);
    this->RequestRenderWindowUpdate();
  }
}

void SurgicalSimulate::OnRobotCapture()
{
  if (m_IndexOfRobotCapture < 5) //The first five translations, 
  {
    CapturePose(true);
    //Increase the count each time you click the button
    m_IndexOfRobotCapture++;
    MITK_INFO << "OnRobotCapture: " << m_IndexOfRobotCapture;
  }
  else if (m_IndexOfRobotCapture < 10) //the last five rotations
  {
    CapturePose(false);
    //Increase the count each time you click the button
    m_IndexOfRobotCapture++;
    MITK_INFO << "OnRobotCapture: " << m_IndexOfRobotCapture;
  }
  else
  {
	MITK_INFO << "OnRobotCapture finish: " << m_IndexOfRobotCapture;
    vtkMatrix4x4* matrix4x4 = vtkMatrix4x4::New();
    m_RobotRegistration.GetRegistraionMatrix(matrix4x4);
    
    

    //For Test Use ,4L tka device registration result ,you can skip registration workflow by using it, Only if the RobotBase Reference Frame not moved!
    /*vtkMatrix4x4* matrix4x4 = vtkMatrix4x4::New();
    matrix4x4->SetElement(0, 0, -0.48); matrix4x4->SetElement(0, 1, -0.19); matrix4x4->SetElement(0, 2, -0.86);
    matrix4x4->SetElement(1, 0, -0.01); matrix4x4->SetElement(1, 1, -0.97); matrix4x4->SetElement(1, 2, 0.22);
    matrix4x4->SetElement(2, 0, -0.88); matrix4x4->SetElement(2, 1, 0.11); matrix4x4->SetElement(2, 2, 0.46);
    matrix4x4->SetElement(0, 3, 162.37);
    matrix4x4->SetElement(1, 3, -530.45);
    matrix4x4->SetElement(2, 3, -255.62);*/

    m_RobotRegistrationMatrix = mitk::AffineTransform3D::New();

    mitk::TransferVtkMatrixToItkTransform(matrix4x4, m_RobotRegistrationMatrix.GetPointer());
    m_VegaToolStorage->GetToolByName("RobotBaseRF")->SetToolRegistrationMatrix(m_RobotRegistrationMatrix);

	MITK_INFO << "Robot Registration Matrix";
	MITK_INFO << m_RobotRegistrationMatrix;
    //build ApplyDeviceRegistrationFilter
    m_KukaApplyRegistrationFilter = lancet::ApplyDeviceRegistratioinFilter::New();
    m_KukaApplyRegistrationFilter->ConnectTo(m_KukaSource);
    m_KukaApplyRegistrationFilter->SetRegistrationMatrix(m_RobotRegistrationMatrix);
    m_KukaApplyRegistrationFilter->SetNavigationDataOfRF(m_VegaSource->GetOutput("RobotBaseRF"));//must make sure NavigationDataOfRF update somewhere else.


    m_KukaVisualizeTimer->stop();
    m_KukaVisualizer->ConnectTo(m_KukaApplyRegistrationFilter);
    m_KukaVisualizeTimer->start();
  }
}

void SurgicalSimulate::OnAutoMove()
{
  //Use current pose as reference to generate new poses
  mitk::NavigationData::Pointer nd_robot2flange = m_KukaSource->GetOutput(0);
  MITK_INFO << "nd_robot2flange";
  MITK_INFO << nd_robot2flange;
  mitk::AffineTransform3D::Pointer affine = mitk::AffineTransform3D::New();
  affine = nd_robot2flange->GetAffineTransform3D()->Clone();
  double axisx[3]{1, 0, 0};
  double axisy[3]{0, 1, 0};
  double axisz[3]{0, 0, 1};
  vtkMatrix4x4* vtkMatrix = vtkMatrix4x4::New();

  double trans1[3]{0, 0, 50};
  double trans2[3]{0, 50, 0};
  double trans3[3]{50, 0, 0};
  double trans4[3]{0, 0, -50};
  double trans5[3]{-25, 0, 0};
  double trans6[3]{0, -25, 0};
  double trans7[3]{0, -25, 0};
  double trans8[3]{25, 0, 0};
  double trans9[3]{-25, 0, 0};
  switch (m_IndexOfRobotCapture)
  {
  case 1: //z+50

    affine->Translate(trans1);
    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  case 2: //y+50


    affine->Translate(trans2);

    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  case 3: //x+50


    affine->Translate(trans3);

    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  case 4: //z-50


    affine->Translate(trans4);
    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  case 5: //x rotate 10 degree x -25


    affine->Rotate3D(axisx, 0.174);
    affine->Translate(trans5);

    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  case 6: //x rotate -20 degree y -25

    affine->Rotate3D(axisx, -0.174 * 2);
    affine->Translate(trans6);

    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  case 7: //y rotate 10 degree y +25

    affine->Rotate3D(axisy, 0.174);
    affine->Translate(trans7);
    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  case 8: //y rotate -20 degree x +25

    affine->Rotate3D(axisy, -0.174 * 2);
    affine->Translate(trans8);
    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  case 9: //z rotate 20 degree x -25

    affine->Rotate3D(axisz, 0.174 * 2);
    affine->Translate(trans9);
    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;

  default:

    mitk::TransferItkTransformToVtkMatrix(affine.GetPointer(), vtkMatrix);

    m_KukaTrackingDevice->RobotMove(vtkMatrix);
    break;
  }
}

void SurgicalSimulate::OnResetRobotRegistration()
{
  m_RobotRegistration.RemoveAllPose();
  m_IndexOfRobotCapture = 0;
}

void SurgicalSimulate::OnCaptureProbeAsSurgicalPlane()
{
  //create NavigationDataToPointSetFilter to get a point3D by probe in NDI coordinates
  mitk::NavigationDataToPointSetFilter::Pointer probePoint = mitk::NavigationDataToPointSetFilter::New();
  //auto probeToolIndex = m_VegaToolStorage->GetToolIndexByName("Probe");
  probePoint->SetInput(m_VegaSource->GetOutput("Probe"));
  probePoint->SetOperationMode(mitk::NavigationDataToPointSetFilter::Mode3DMean);
  probePoint->SetNumberForMean(10);
  //run the filter
  probePoint->Update();
  //get output
  mitk::PointSet::Pointer target = probePoint->GetOutput(0);

  // //create Surgaical plane
  // m_SurgicalPlan = lancet::PointPath::New();
  //convert to robot coordinates
  mitk::AffineTransform3D::Pointer targetMatrix = mitk::AffineTransform3D::New();
  targetMatrix->SetOffset(target->GetPoint(0).GetDataPointer());
  MITK_INFO << "Captured Point: " << targetMatrix;

  m_T_robot = mitk::AffineTransform3D::New();
  m_VegaSource->SetToolMetaDataCollection(m_VegaToolStorage);
  m_VegaSource->TransferCoordsFromTrackingDeviceToTrackedObject("RobotBaseRF", targetMatrix, m_T_robot);

  //use robot matrix,not change the end tool rotation,only apply the offset from probe;
  m_T_robot->SetMatrix(m_KukaSource->GetOutput(0)->GetAffineTransform3D()->GetMatrix());

  MITK_INFO << m_T_robot;
}

void SurgicalSimulate::OnAutoPositionStart()
{
  vtkMatrix4x4* t = vtkMatrix4x4::New();
  mitk::TransferItkTransformToVtkMatrix(m_T_robot.GetPointer(), t);
  
  
  m_KukaTrackingDevice->RobotMove(t);
}

void SurgicalSimulate::UseVirtualDevice1()
{
  //read in filename
  QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Tool Storage"), "/",
                                                  tr("Tool Storage Files (*.IGTToolStorage)"));
  if (filename.isNull()) return;

  //read tool storage from disk
  std::string errorMessage = "";
  mitk::NavigationToolStorageDeserializer::Pointer myDeserializer = mitk::NavigationToolStorageDeserializer::New(
    GetDataStorage());
  m_VirtualDevice1ToolStorage = myDeserializer->Deserialize(filename.toStdString());
  m_VirtualDevice1ToolStorage->SetName(filename.toStdString());

  //! [UseKuka 1]
  //Here we want to use the VirtualDevice as a tracking device. Therefore we instantiate a object of the class
  //KukaRobotDevice and make some settings which are necessary for a proper connection to the device.
  MITK_INFO << "VirtualDevice1 tracking";
  //QMessageBox::warning(nullptr, "Warning", "You have to set the parameters for the NDITracking device inside the code (QmitkIGTTutorialView::OnStartIGT()) before you can use it.");
  m_VirtualDevice1 = mitk::VirtualTrackingDevice::New(); //instantiate

  //Create Navigation Data Source with the factory class, and the visualize filter.
  lancet::TrackingDeviceSourceConfiguratorLancet::Pointer kukaSourceFactory =
    lancet::TrackingDeviceSourceConfiguratorLancet::New(m_VirtualDevice1ToolStorage, m_VirtualDevice1);

  m_VirtualDevice1Source = kukaSourceFactory->CreateTrackingDeviceSource(m_VirtualDevice1Visualizer);

  m_VirtualDevice1Source->Connect();

  m_VirtualDevice1Source->StartTracking();

  //update visualize filter by timer
  if (m_VirtualDevice1Timer == nullptr)
  {
    m_VirtualDevice1Timer = new QTimer(this); //create a new timer
  }
  connect(m_VirtualDevice1Timer, SIGNAL(timeout()), this, SLOT(OnVirtualDevice1VisualizeTimer()));
  //connect the timer to the method OnTimer()
  connect(m_VirtualDevice1Timer, SIGNAL(timeout()), this, SLOT(UpdateToolStatusWidget()));
  //connect the timer to the method OnTimer()
  ShowToolStatus_Vega();

  m_VirtualDevice1Timer->start(100); //Every 100ms the method OnTimer() is called. -> 10fps
  auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
  mitk::RenderingManager::GetInstance()->InitializeViews(geo);
}

void SurgicalSimulate::OnVirtualDevice1VisualizeTimer()
{
  //Here we call the Update() method from the Visualization Filter. Internally the filter checks if
  //new NavigationData is available. If we have a new NavigationData the cone position and orientation
  //will be adapted.
  if (m_VirtualDevice1Visualizer.IsNotNull())
  {
    m_VirtualDevice1Visualizer->Update();
    // auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
    // mitk::RenderingManager::GetInstance()->InitializeViews(geo);
    this->RequestRenderWindowUpdate();
  }
}

void SurgicalSimulate::UseVirtualDevice2()
{
  //read in filename
  QString filename = QFileDialog::getOpenFileName(nullptr, tr("Open Tool Storage"), "/",
                                                  tr("Tool Storage Files (*.IGTToolStorage)"));
  if (filename.isNull()) return;

  //read tool storage from disk
  std::string errorMessage = "";
  mitk::NavigationToolStorageDeserializer::Pointer myDeserializer = mitk::NavigationToolStorageDeserializer::New(
    GetDataStorage());
  m_VirtualDevice2ToolStorage = myDeserializer->Deserialize(filename.toStdString());
  m_VirtualDevice2ToolStorage->SetName(filename.toStdString());

  //! [UseKuka 1]
  //Here we want to use the VirtualDevice as a tracking device. Therefore we instantiate a object of the class
  //KukaRobotDevice and make some settings which are necessary for a proper connection to the device.
  MITK_INFO << "VirtualDevice1 tracking";
  //QMessageBox::warning(nullptr, "Warning", "You have to set the parameters for the NDITracking device inside the code (QmitkIGTTutorialView::OnStartIGT()) before you can use it.");
  m_VirtualDevice2 = mitk::VirtualTrackingDevice::New(); //instantiate

  //Create Navigation Data Source with the factory class, and the visualize filter.
  lancet::TrackingDeviceSourceConfiguratorLancet::Pointer kukaSourceFactory =
    lancet::TrackingDeviceSourceConfiguratorLancet::New(m_VirtualDevice2ToolStorage, m_VirtualDevice2);

  m_VirtualDevice2Source = kukaSourceFactory->CreateTrackingDeviceSource(m_VirtualDevice2Visualizer);

  m_VirtualDevice2Source->Connect();

  m_VirtualDevice2Source->StartTracking();

  //update visualize filter by timer
  if (m_VirtualDevice2Timer == nullptr)
  {
    m_VirtualDevice2Timer = new QTimer(this); //create a new timer
  }
  connect(m_VirtualDevice2Timer, SIGNAL(timeout()), this, SLOT(OnVirtualDevice2VisualizeTimer()));
  //connect the timer to the method OnTimer()
  connect(m_VirtualDevice2Timer, SIGNAL(timeout()), this, SLOT(UpdateToolStatusWidget()));
  //connect the timer to the method OnTimer()
  ShowToolStatus_Kuka();
  m_VirtualDevice2Timer->start(100); //Every 100ms the method OnTimer() is called. -> 10fps

  auto geo = this->GetDataStorage()->ComputeBoundingGeometry3D(this->GetDataStorage()->GetAll());
  mitk::RenderingManager::GetInstance()->InitializeViews(geo);
}

void SurgicalSimulate::UpdateToolStatusWidget()
{
  m_Controls.m_StatusWidgetVegaToolToShow->Refresh();
  m_Controls.m_StatusWidgetKukaToolToShow->Refresh();
}

void SurgicalSimulate::ShowToolStatus_Vega()
{
  m_VegaNavigationData.clear();
  for (std::size_t i = 0; i < m_VegaSource->GetNumberOfOutputs(); i++)
  {
    m_VegaNavigationData.push_back(m_VegaSource->GetOutput(i));
  }
  //initialize widget
  m_Controls.m_StatusWidgetVegaToolToShow->RemoveStatusLabels();
  m_Controls.m_StatusWidgetVegaToolToShow->SetShowPositions(true);
  m_Controls.m_StatusWidgetVegaToolToShow->SetTextAlignment(Qt::AlignLeft);
  m_Controls.m_StatusWidgetVegaToolToShow->SetNavigationDatas(&m_VegaNavigationData);
  m_Controls.m_StatusWidgetVegaToolToShow->ShowStatusLabels();
}

void SurgicalSimulate::ShowToolStatus_Kuka()
{
  m_KukaNavigationData.clear();
  for (std::size_t i = 0; i < m_KukaSource->GetNumberOfOutputs(); i++)
  {
    m_KukaNavigationData.push_back(m_KukaSource->GetOutput(i));
  }
  //initialize widget
  m_Controls.m_StatusWidgetKukaToolToShow->RemoveStatusLabels();
  m_Controls.m_StatusWidgetKukaToolToShow->SetShowPositions(true);
  m_Controls.m_StatusWidgetKukaToolToShow->SetTextAlignment(Qt::AlignLeft);
  m_Controls.m_StatusWidgetKukaToolToShow->SetNavigationDatas(&m_KukaNavigationData);
  m_Controls.m_StatusWidgetKukaToolToShow->ShowStatusLabels();
}

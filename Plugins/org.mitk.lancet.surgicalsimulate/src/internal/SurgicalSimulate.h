/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef SurgicalSimulate_h
#define SurgicalSimulate_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>


#include "kukaRobotDevice.h"
#include "mitkVirtualTrackingDevice.h"
#include "mitkVirtualTrackingTool.h"
#include "lancetNavigationObjectVisualizationFilter.h"
#include "lancetApplyDeviceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinFilter.h"
#include "lancetApplySurfaceRegistratioinStaticImageFilter.h"
#include "lancetPathPoint.h"
#include "mitkTrackingDeviceSource.h"
#include "robotRegistration.h"
#include "ui_SurgicalSimulateControls.h"

/**
  \brief SurgicalSimulate

  \warning  This class is not yet documented. Use "git blame" and ask the author to provide basic documentation.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class SurgicalSimulate : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;


public slots:
  //Step1:Use a kuka Tracking Device
  void UseKuka();
  //
  void StartTracking();

  // EndTool power control
  void StartTrackingWithPowerControl();
  void OnCheckPowerStatusTimer();
 

  void OnKukaVisualizeTimer();
  void OnSelfCheck();
  //Step1:Use a NDI Vega Tracking Device
  void UseVega();
  void OnVegaVisualizeTimer();

  //Step2:Robot Registration;
  void OnRobotCapture();
  void OnAutoMove();
  void OnResetRobotRegistration();
  void OnSaveRobotRegistraion();
  void OnUsePreRobotRegitration();

  //Step3:NavigationObject Registration

  //Step4: Create Surgical Plane
  // void OnCaptureProbeAsSurgicalPlane();

  //Step5: Run the Surgical Plane by robot auto move to position
  void OnAutoPositionStart();

  void On_pushButton_goToFakePlane_clicked();


  //TEST
  void UseVirtualDevice1();
  void OnVirtualDevice1VisualizeTimer();
  void UseVirtualDevice2();
  void OnVirtualDevice2VisualizeTimer();

  //ShowToolStatus
  void ShowToolStatus_Vega();
  void ShowToolStatus_Kuka();
  void UpdateToolStatusWidget();


  // Robot tests written by Huanyu Li
  void on_pushButton_robotCube_clicked(); // 100 mm X 100 mm X 100 mm Cube
  void on_pushButton_robotTwoPts_clicked();
  void on_pushButton_thaMaxSpace_clicked();
  void on_pushButton_thaEffectiveSpace_clicked();
  void on_pushButton_tkaMaxSpace_clicked();
  void on_pushButton_tkaEffectiveSpace_clicked();
  void on_pushButton_robotEmergencyBrake_clicked();

  // 2024.08.06 THA image registration optimization


protected:
  virtual void CreateQtPartControl(QWidget* parent) override;

  virtual void SetFocus() override;

  /// \brief called by QmitkFunctionality when DataManager's selection has changed
  virtual void OnSelectionChanged(berry::IWorkbenchPart::Pointer source,
                                  const QList<mitk::DataNode::Pointer>& nodes) override;


  // [Step2 Robot Registration]
  /// \brief Generate poses that robot needs to move for registration.
  ///We take the present pose of the robot arm as the initial pose, first translating five poses, and then moving five poses with rotation.
  void GeneratePoses();

  void CapturePose(bool translationOnly);


  //*********Helper Function****************
  RobotRegistration m_RobotRegistration;
  mitk::NavigationData::Pointer GetNavigationDataInRef(mitk::NavigationData::Pointer nd,
                                                       mitk::NavigationData::Pointer nd_ref);
public:
  ~SurgicalSimulate() override;
protected:
  Ui::SurgicalSimulateControls m_Controls;


  lancet::KukaRobotDevice::Pointer m_KukaTrackingDevice;
  //vega trackingDeviceSource
  mitk::TrackingDeviceSource::Pointer m_VegaSource;
  //kuka trackingDeviceSource
  mitk::TrackingDeviceSource::Pointer m_KukaSource;

  lancet::NavigationObjectVisualizationFilter::Pointer m_KukaVisualizer;
  lancet::NavigationObjectVisualizationFilter::Pointer m_VegaVisualizer;
  lancet::ApplyDeviceRegistratioinFilter::Pointer m_KukaApplyRegistrationFilter;
  QTimer* m_KukaVisualizeTimer{nullptr};
  QTimer* m_VegaVisualizeTimer{nullptr};
  mitk::NavigationToolStorage::Pointer m_KukaToolStorage;
  mitk::NavigationToolStorage::Pointer m_VegaToolStorage;

  //robot registration
  unsigned int m_IndexOfRobotCapture{0};
  std::array<vtkMatrix4x4*, 10> m_AutoPoses{};
  mitk::AffineTransform3D::Pointer m_RobotRegistrationMatrix;

  //surgical plane
  lancet::PointPath::Pointer m_SurgicalPlan;
  mitk::AffineTransform3D::Pointer m_T_robot;

  //filter test
  mitk::VirtualTrackingDevice::Pointer m_VirtualDevice1;
  mitk::VirtualTrackingDevice::Pointer m_VirtualDevice2;
  mitk::TrackingDeviceSource::Pointer m_VirtualDevice1Source;
  mitk::TrackingDeviceSource::Pointer m_VirtualDevice2Source;
  lancet::NavigationObjectVisualizationFilter::Pointer m_VirtualDevice1Visualizer;
  lancet::NavigationObjectVisualizationFilter::Pointer m_VirtualDevice2Visualizer;
  QTimer* m_VirtualDevice1Timer{nullptr};
  QTimer* m_VirtualDevice2Timer{nullptr};
  mitk::NavigationToolStorage::Pointer m_VirtualDevice1ToolStorage;
  mitk::NavigationToolStorage::Pointer m_VirtualDevice2ToolStorage;
  QTimer* m_ToolStatusTimer{nullptr}; //<<< tracking timer that updates the status widgets

  std::vector<mitk::NavigationData::Pointer> m_VegaNavigationData;
  std::vector<mitk::NavigationData::Pointer> m_KukaNavigationData;



  // Image registration using NavigationObject structure
 
	// Assemble a navigationObject with a Parent node;
	// the name of the Parent node becomes the ReferencFrameName of the navigationObject
	// the Parent node contains the surface for visualization and the following sub-nodes:
	// 1. "surfaceToRf_matrix": optional; surface frame to reference frame transformation;
	//   uses a point-set containing 4 points (4 columns of the matrix) to represent;
	//   if empty, construct an identity matrix as default in NavigationObject and a subNode point-set as well;
	// 2. "surface_backup": optional; a backup of the original surface data;
	//   if empty, duplicate a surface as a subNode
	// 3. "landmark_surface": compulsory, landmark points on the surface in the surface frame
	//   if empty, prompt warning
	// 4. "landmark_rf": optional, collected landmark points in the reference frame;
	//   if empty, create an empty point-set in NavigationObject
	// 5. "icp_surface": optional, icp points on the surface in the surface frame
	//   if empty, do nothing
	// 6. "icp_rf": optional, collected icp points in the reference frame, only for visualization purposes;
	//   if empty, do nothing
  bool AssembleNavigationObjectFromDataNode(mitk::DataNode* parentNode, lancet::NavigationObject* assembledObject);

  bool RetoreDataNodeFromNavigationObject(mitk::DataNode* parentNode, lancet::NavigationObject* assembledObject);

  lancet::ApplySurfaceRegistratioinFilter::Pointer m_surfaceRegistrationFilter;
  lancet::ApplySurfaceRegistratioinStaticImageFilter::Pointer m_surfaceRegistrationStaticImageFilter;

  void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
  void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);

  lancet::NavigationObject::Pointer navigatedImage;
  mitk::AffineTransform3D::Pointer m_imageRegistrationMatrix; // image(surface) to Rf matrix


  bool SetupNavigatedImage();
  bool CollectLandmarkProbe();
  bool CollectIcpProbe();
  bool ApplySurfaceRegistration();
  bool ApplyPreexistingImageSurfaceRegistration();

  bool ApplySurfaceRegistration_staticImage();
  bool ApplyPreexistingImageSurfaceRegistration_staticImage();
	
  // Get the coordinate of the image line in the robot (internal) base frame
  bool InterpretImageLine();

  // Get the coordinate of the image plane in the robot (internal) base frame
  bool InterpretImagePlane();

  // Use butterfly to calibrate the probe
  bool TouchProbeCalibrationPoint1();
  bool TouchProbeCalibrationPoint2();
  bool TouchProbeCalibrationPoint3();
  double m_probeOffset[3]{ 0,0,0 };

  // Collect and evaluate image checkPoint after image registration
  bool ProbeImageCheckPoint();

  // Check the distance between the probe and image after image registration
  bool ProbeSurface();



  bool AverageNavigationData(mitk::NavigationData::Pointer ndPtr, int timeInterval /*milisecond*/, int intervalNum, double matrixArray[16]);
  vtkMatrix4x4* getVtkMatrix4x4(mitk::NavigationData::Pointer nd);


  bool ResetRobotTcp();
  bool SetPrecisionTestTcp();
  bool SetPlanePrecisionTestTcp();


	// Move Kuka robot
  vtkMatrix4x4* m_initial_robotBaseToFlange;
  bool RecordInitial();
  bool GoToInitial();
  bool InterpretMovementAsInBaseSpace(vtkMatrix4x4* rawMovementMatrix, vtkMatrix4x4* movementMatrixInRobotBase);
  /// Kuka translation
  bool TranslateX_plus();
  bool TranslateY_plus();
  bool TranslateZ_plus();
  bool TranslateX_minus();
  bool TranslateY_minus();
  bool TranslateZ_minus();

  /// Kuka rotation
  bool RotateX_plus();
  bool RotateY_plus();
  bool RotateZ_plus();
  bool RotateX_minus();
  bool RotateY_minus();
  bool RotateZ_minus();

  // Simulate robot pose
  void On_pushButton_updateRobotSimuPose_clicked();
  void On_pushButton_a1_p_clicked();
  void On_pushButton_a1_m_clicked();
  void On_pushButton_a2_p_clicked();
  void On_pushButton_a2_m_clicked();
  void On_pushButton_a3_p_clicked();
  void On_pushButton_a3_m_clicked();
  void On_pushButton_a4_p_clicked();
  void On_pushButton_a4_m_clicked();
  void On_pushButton_a5_p_clicked();
  void On_pushButton_a5_m_clicked();
  void On_pushButton_a6_p_clicked();
  void On_pushButton_a6_m_clicked();
  void On_pushButton_a7_p_clicked();
  void On_pushButton_a7_m_clicked();

  // EndTool power control
  QTimer* m_CheckPowerStatusTimer{nullptr};
  int m_PowerStatus{ 0 };


  // THA registration optimization 2024.08.06
  mitk::PointSet::Pointer m_Pset_thaCrest_rf = mitk::PointSet::New();
  mitk::PointSet::Pointer m_Pset_thaPosHorn_rf = mitk::PointSet::New();
  mitk::PointSet::Pointer m_Pset_thaAntHorn_rf = mitk::PointSet::New();
  mitk::PointSet::Pointer m_Pset_thaArtiSurface_rf = mitk::PointSet::New();
  mitk::PointSet::Pointer m_Pset_thaExArtiSurface_rf = mitk::PointSet::New();

  mitk::PointSet::Pointer m_Pset_thaBoneLandmark = mitk::PointSet::New();
  mitk::Surface::Pointer m_Surface_thaBoneSurface = mitk::Surface::New();

  vtkNew<vtkMatrix4x4> m_ObjectRfToImageMatrix_tha;

  void on_pushButton_initThaRegis_clicked();
  void on_pushButton_tha_crest_clicked();
  void on_pushButton_tha_articular_clicked();
  void on_pushButton_tha_posHorn_clicked();
  void on_pushButton_tha_antHorn_clicked();
  void on_pushButton_tha_extraArticular_clicked();
  // void on_pushButton_tha_regisReset_clicked();

  void ThaSwitchStateMachine();

  bool ThaRegister(); /* Do following tasks:
	- check icp & landmark & surface availability, return false if all data are not ready
	- Calculate registration, update m_ObjectRfToImageMatrix_tha and return true; */

  int ThaCheckLandmarkQuality();/* Check landmark quality:
	- Return 0 if both pos & ant horn are okay
	- Return 1 if pos horn is problematic
	- Return 2 if only ant horn is problematic
	- Return 3 if ThaRegister() failed*/

  void ThaDisplayProblematicLandmark(int landmarkIndex); /* Display the problematic landmark with its image counterpart:
	1: posterior horn
	2: anterior horn
	this function co-works with ThaCheckLandmarkQuality()*/

  void ThaDisplayIcpPts();
};

#endif // SurgicalSimulate_h

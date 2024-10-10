#ifndef ROBOTREGISTRATION_H
#define ROBOTREGISTRATION_H

#include <itkObject.h>
#include <itkObjectFactory.h>
#include <mitkCommon.h>
#include "MitkLancetRobotExports.h"

#include <vector>
#include <mitkNavigationData.h>

class vtkMatrix4x4;
class MITKLANCETROBOT_EXPORT RobotRegistration:public itk::Object
{

public:
	
	mitkClassMacroItkParent(RobotRegistration, itk::Object);
	itkNewMacro(Self);

	/**
	 * \brief Input required data for registration.
	 * \param nd_robot2flange Pose matrix of robot arm: Namely, the transformation matrix from the robot base to the flange coordinates.
	 * \param nd_Ndi2RegistFrame The coordinates of the end tool, in the reference coordinates if there is a reference marker.
	 *
	 * \param translationOnly Indicates that the current capture of robot arm movement is translational only.
	 */
	void AddPose(mitk::NavigationData::Pointer nd_robot2flange, mitk::NavigationData::Pointer nd_Ndi2RegistFrame, bool translationOnly);
	void AddPoseWithVtkMatrix(vtkMatrix4x4* vtkRoboBaseToFlange, vtkMatrix4x4* vtkRobotBaseRF2RobotEndRF, bool translationOnly);
	bool PopLastPose();
	bool PopLastPose(bool translationOnly);
	void RemoveAllPose();

	int PoseCount() const;

	/**
	 * \brief Get the Registration Matrix.the transform matrix from robot marker to robot base(Tm2b)
	 */

	bool GetRegistraionMatrix(vtkMatrix4x4* output);

	void GetTCP(std::array<double, 6>& output);

	void GetTCPmatrix(vtkMatrix4x4* flangeToRoboEndToMatrix);

	bool Regist();

	double RMS();

	void Print();


	// step 1
	void calculateR0();

	// step 2
	void calculateVe();

	// step 3
	void calculateV0();

	// step 4
	void calculateRe();

	void genTestData();

	/**
	 * \brief endTool R in RobotMarker.
	 */
	std::vector<Eigen::Matrix3d> Rn{};
	/**
	 * \brief endTool V in RobotMarker.
	 */
	std::vector<Eigen::Vector3d> Vn{};

	/**
	 * \brief robot registration matrix,from RobotMaker to robot base.
	 */
	Eigen::Matrix3d R0{};
	/**
	 * \brief robot registration matrix,from RobotMaker to robot base.
	 */
	Eigen::Vector3d V0{};

	/**
	 * \brief Transform matrix of Robot flange in robot base;
	 */
	std::vector<Eigen::Matrix3d> R{};
	std::vector<Eigen::Vector3d> V{};

	/**
	 * \brief TCP rotation,from flange to endTool.
	 */
	Eigen::Matrix3d Re{};
	/**
	 * \brief TCP displacement,from flange to endTool.
	 */
	Eigen::Vector3d Ve{};

	/**
	 * \brief Used to indicate that the first n values in the vector are not changing the robot attitude
	 */
	int m_numberOfFixR{ 0 };

	int m_numberOfPose{ 0 };

	bool m_translationOnly{ true };

	bool m_calculateEndToolAttitude{ false };

};

#endif

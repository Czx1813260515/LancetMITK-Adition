#include "robotRegistration.h"
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <mitkLog.h>

#include "mitkPointSet.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace Eigen;
void RobotRegistration::AddPose(mitk::NavigationData::Pointer nd_robot2flange, mitk::NavigationData::Pointer nd_Ndi2RegistFrame, bool translationOnly)
{
  mitk::Matrix3D Rrobot2flange = nd_robot2flange->GetRotationMatrix();
  mitk::Point3D Vrobot2flange = nd_robot2flange->GetPosition();
  mitk::Matrix3D Rndi2RegistFrame = nd_Ndi2RegistFrame->GetRotationMatrix();
  mitk::Point3D Vndi2RegistFrame = nd_Ndi2RegistFrame->GetPosition();

	Matrix3d r;
	r << Rrobot2flange(0, 0), Rrobot2flange(0, 1), Rrobot2flange(0, 2), Rrobot2flange(1, 0),
		Rrobot2flange(1, 1), Rrobot2flange(1, 2), Rrobot2flange(2, 0), Rrobot2flange(2, 1),
		Rrobot2flange(2, 2);

	Vector3d v;
	v << Vrobot2flange[0], Vrobot2flange[1], Vrobot2flange[2];

	Matrix3d rn;
	rn << Rndi2RegistFrame(0, 0), Rndi2RegistFrame(0, 1), Rndi2RegistFrame(0, 2), Rndi2RegistFrame(1, 0),
		Rndi2RegistFrame(1, 1), Rndi2RegistFrame(1, 2), Rndi2RegistFrame(2, 0), Rndi2RegistFrame(2, 1),
		Rndi2RegistFrame(2, 2);

	Vector3d vn;
    vn << Vndi2RegistFrame[0], Vndi2RegistFrame[1], Vndi2RegistFrame[2];

	this->m_translationOnly = translationOnly;
	if (translationOnly)
	{
		R.emplace(R.begin(), r);
		V.emplace(V.begin(), v);
		Rn.emplace(Rn.begin(), rn);
		Vn.emplace(Vn.begin(), vn);
		m_numberOfFixR++;
	}
	else
	{
		R.push_back(r);
		V.push_back(v);
		Rn.push_back(rn);
		Vn.push_back(vn);
	}
	m_numberOfPose++;
}

void RobotRegistration::AddPoseWithVtkMatrix(vtkMatrix4x4* vtkRoboBaseToFlange, vtkMatrix4x4* vtkRobotBaseRF2RobotEndRF, bool translationOnly)
{
	Matrix3d r;
	r << vtkRoboBaseToFlange->GetElement(0, 0), vtkRoboBaseToFlange->GetElement(0, 1), vtkRoboBaseToFlange->GetElement(0, 2),
		vtkRoboBaseToFlange->GetElement(1, 0), vtkRoboBaseToFlange->GetElement(1, 1), vtkRoboBaseToFlange->GetElement(1, 2),
		vtkRoboBaseToFlange->GetElement(2, 0), vtkRoboBaseToFlange->GetElement(2, 1), vtkRoboBaseToFlange->GetElement(2, 2);
	
	Vector3d v;
	v << vtkRoboBaseToFlange->GetElement(0, 3), vtkRoboBaseToFlange->GetElement(1, 3), vtkRoboBaseToFlange->GetElement(2, 3);

	Matrix3d rn;
	rn << vtkRobotBaseRF2RobotEndRF->GetElement(0, 0), vtkRobotBaseRF2RobotEndRF->GetElement(0, 1), vtkRobotBaseRF2RobotEndRF->GetElement(0, 2),
		vtkRobotBaseRF2RobotEndRF->GetElement(1, 0), vtkRobotBaseRF2RobotEndRF->GetElement(1, 1), vtkRobotBaseRF2RobotEndRF->GetElement(1, 2),
		vtkRobotBaseRF2RobotEndRF->GetElement(2, 0), vtkRobotBaseRF2RobotEndRF->GetElement(2, 1), vtkRobotBaseRF2RobotEndRF->GetElement(2, 2);


	Vector3d vn;
	vn << vtkRobotBaseRF2RobotEndRF->GetElement(0, 3), vtkRobotBaseRF2RobotEndRF->GetElement(1, 3), vtkRobotBaseRF2RobotEndRF->GetElement(2, 3);

	this->m_translationOnly = translationOnly;
	if (translationOnly)
	{
		R.emplace(R.begin(), r);
		V.emplace(V.begin(), v);
		Rn.emplace(Rn.begin(), rn);
		Vn.emplace(Vn.begin(), vn);
		m_numberOfFixR++;
	}
	else
	{
		R.push_back(r);
		V.push_back(v);
		Rn.push_back(rn);
		Vn.push_back(vn);
	}
	m_numberOfPose++;
}



bool RobotRegistration::PopLastPose()
{
	return this->PopLastPose(this->m_translationOnly);
}

bool RobotRegistration::PopLastPose(bool translationOnly)
{
	if (R.size() != V.size() || V.size() != Rn.size() || Rn.size() != Vn.size())
	{
		MITK_WARN << "Unknown error, the number of internal containers does not correspond.";
		return false;
	}

	if (R.size() == 0)
	{
		MITK_WARN << "Internal container quantity is empty.";
		return false;
	}

	if (true == translationOnly)
	{
		R.erase(R.begin());
		V.erase(V.begin());
		Rn.erase(Rn.begin());
		Vn.erase(Vn.begin());
		--m_numberOfFixR;
	}
	else
	{
		R.pop_back();
		V.pop_back();
		Rn.pop_back();
		Vn.pop_back();
	}
	--m_numberOfPose;
	return true;
}

void RobotRegistration::RemoveAllPose()
{
	Rn.clear();
	Vn.clear();

	R0 = Matrix3d();
	V0 = Vector3d();

	R.clear();
	V.clear();

	Re = Matrix3d();
	Ve = Vector3d();

	m_numberOfFixR = 0;
	m_numberOfPose = 0;

	m_translationOnly = true;
	m_calculateEndToolAttitude = false;
}

int RobotRegistration::PoseCount() const
{
	return m_numberOfPose;
}

bool RobotRegistration::GetRegistraionMatrix(vtkMatrix4x4* output)
{
	if (Regist() == false)
	{
		return false;
	}

	Matrix4d matrix;
	matrix.setIdentity();
	matrix.block(0, 0, 3, 3) = R0;
	matrix.block(0, 3, 3, 1) = V0;

	vtkNew<vtkMatrix4x4> res;
	res->DeepCopy(matrix.data());
	res->Transpose();

	// Exclude the spacing component to ensure the 3x3 matrix is a pure rotation matrix
	auto tmpPset = mitk::PointSet::New();
	tmpPset->GetGeometry()->SetIndexToWorldTransformByVtkMatrixWithoutChangingSpacing(res);
	auto res_withoutSpacing = tmpPset->GetGeometry()->GetVtkMatrix();

	output->DeepCopy(res_withoutSpacing);
	return true;
}

void RobotRegistration::GetTCP(std::array<double, 6>& output)
{
	//In case anyone forgets to do calculation
	Regist();

	output[0] = Ve(0);
	output[1] = Ve(1);
	output[2] = Ve(2);

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	output[3] = eulerAngle(0);
	output[4] = eulerAngle(1);
	output[5] = eulerAngle(2);
}

void RobotRegistration::GetTCPmatrix(vtkMatrix4x4* flangeToRoboEndToMatrix)
{
	//In case anyone forgets to do calculation
	Regist();

	flangeToRoboEndToMatrix->Identity();
	flangeToRoboEndToMatrix->SetElement(0, 0, Re(0, 0));
	flangeToRoboEndToMatrix->SetElement(0, 1, Re(0, 1));
	flangeToRoboEndToMatrix->SetElement(0, 2, Re(0, 2));
	flangeToRoboEndToMatrix->SetElement(1, 0, Re(1, 0));
	flangeToRoboEndToMatrix->SetElement(1, 1, Re(1, 1));
	flangeToRoboEndToMatrix->SetElement(1, 2, Re(1, 2));
	flangeToRoboEndToMatrix->SetElement(2, 0, Re(2, 0));
	flangeToRoboEndToMatrix->SetElement(2, 1, Re(2, 1));
	flangeToRoboEndToMatrix->SetElement(2, 2, Re(2, 2));

	flangeToRoboEndToMatrix->SetElement(0, 3, Ve(0));
	flangeToRoboEndToMatrix->SetElement(1, 3, Ve(1));
	flangeToRoboEndToMatrix->SetElement(2, 3, Ve(2));

}



bool RobotRegistration::Regist()
{
	//check calculation pre requirements
	if (m_numberOfFixR < 4)
	{
		MITK_ERROR << "At least four translation only pose needed to regist";
		return false;
	}
	// check if truly translate only
	for (int i = 0; i < m_numberOfFixR; i++)
	{
		if (!R[i].isApprox(R[0], 0.001))
		{
			MITK_WARN << "There is rotation in Only translation pose, Registration Is not accurate. ";
			MITK_WARN << R[0];
			MITK_WARN << R[i];
		}
	}
	//print r v
	MITK_INFO << "RV robot ";
	for each (auto r in R)
	{
		MITK_INFO << r;
	}
	for each (auto v in V)
	{
		MITK_INFO << v;
	}
	MITK_INFO << "RV ndi base";
	for each (auto rn in Rn)
	{
		MITK_INFO << rn;
	}
	for each (auto vn in Vn)
	{
		MITK_INFO << vn;
	}
	calculateR0();
	calculateVe();
	calculateV0();
	calculateRe();
	return true;
}

double RobotRegistration::RMS()
{
	double res = 0;
	for (int i = 0; i < m_numberOfPose; i++)
	{
		Vector3d err = Vn[i] - R0 * R[i] * Ve - R0 * V[i] - V0;
		res += err.cwiseProduct(err).sum();
	}
	return sqrt(res / m_numberOfPose);
}

void RobotRegistration::Print()
{
	for (int i = 0; i < Rn.size(); i++)
	{
		std::cout << "Robot Matrix in algo" << std::endl;
		std::cout << R[i] << std::endl;
		std::cout << V[i] << std::endl;
		std::cout << "tool Matrix in algo" << std::endl;
		std::cout << Rn[i] << std::endl;
		std::cout << Vn[i] << std::endl;
	}
}

void RobotRegistration::calculateR0()
{
	MatrixXd A;
	A.resize(3, m_numberOfFixR - 1);
	int col_index = 0;
	for (int i = 1; i < m_numberOfFixR; i++)
	{
		Vector3d dV = V[i] - V[i - 1];
		A.col(col_index) = dV;
		col_index++;
	}
	//std::cout << "A:\n" << A << std::endl;

	Matrix3Xd B;
	B.resize(3, m_numberOfFixR - 1);
	col_index = 0;
	for (int i = 1; i < m_numberOfFixR; i++)
	{
		Vector3d dV = Vn[i] - Vn[i - 1];
		B.col(col_index) = dV;
		col_index++;
	}
	//std::cout << "B:\n" << B << std::endl;

	MatrixXd ABt = A * B.transpose();
	//std::cout << "ABt:\n" << ABt << std::endl;

	JacobiSVD<MatrixXd> svd(ABt, ComputeThinU | ComputeThinV);
	auto v = svd.matrixV();
	auto u = svd.matrixU();
	//std::cout << "V:\n" << v << std::endl;
	//std::cout << "U:\n" << u << std::endl;
	// Todo: this matrix often fails to become a pure rotation matrix due to the data collection error.
	// Todo: need to exclude the spacing component
	R0 = v * u.transpose();

	Eigen::Vector3d x;
	Eigen::Vector3d y;
	Eigen::Vector3d z;

	for(int i{0}; i < 3; i++)
	{
		x[i] = R0(i, 0);
		y[i] = R0(i, 1);
		z[i] = R0(i, 2);
	}

	x.normalize();
	y.normalize();
	z.normalize();

	Eigen::Vector3d x_;
	Eigen::Vector3d y_;
	Eigen::Vector3d z_;

	x_ = x;
	z_ = x_.cross(y);
	y_ = z_.cross(x_);

	z_.normalize();
	y_.normalize();

	for (int i{ 0 }; i < 3; i++)
	{
		R0(i, 0) = x_[i];
		R0(i, 1) = y_[i];
		R0(i, 2) = z_[i];
	}

	//std::cout << "R0:\n" << R0 << std::endl;
}

void RobotRegistration::calculateRe()
{
	MatrixXd A;
	const int n = m_numberOfPose - m_numberOfFixR;
	A.resize(3 * n, 3);
	for (int i = 0; i < n; i++)
	{
		A.block(3 * i, 0, 3, 3) = R[i + m_numberOfFixR];
	}
	//std::cout << "A:\n" << A << std::endl;

	MatrixXd B;
	B.resize(3 * n, 3);
	for (int i = 0; i < n; i++)
	{
		B.block(3 * i, 0, 3, 3) = R0.transpose() * Rn[i + m_numberOfFixR];
	}
	//std::cout << "B:\n" << B << std::endl;
	Re = (A.transpose() * A).inverse() * A.transpose() * B;
	//std::cout << "Re:\n" << Re << std::endl;


	Eigen::Vector3d x;
	Eigen::Vector3d y;
	Eigen::Vector3d z;

	for (int i{ 0 }; i < 3; i++)
	{
		x[i] = Re(i, 0);
		y[i] = Re(i, 1);
		z[i] = Re(i, 2);
	}

	x.normalize();
	y.normalize();
	z.normalize();

	Eigen::Vector3d x_;
	Eigen::Vector3d y_;
	Eigen::Vector3d z_;

	x_ = x;
	z_ = x_.cross(y);
	y_ = z_.cross(x_);

	z_.normalize();
	y_.normalize();

	for (int i{ 0 }; i < 3; i++)
	{
		Re(i, 0) = x_[i];
		Re(i, 1) = y_[i];
		Re(i, 2) = z_[i];
	}
}

void RobotRegistration::calculateVe()
{
	MatrixXd A;
	const int n = m_numberOfPose - m_numberOfFixR - 1;
	A.resize(3 * n, 3);
	for (int i = 0; i < n; i++)
	{
		Matrix3d a = R0 * (R[i + m_numberOfFixR + 1] - R[i + m_numberOfFixR]);
		A.block(3 * i, 0, 3, 3) = a;
	}
	//std::cout << "A:\n" << A << std::endl;

	MatrixXd B;
	B.resize(3 * n, 1);
	for (int i = 0; i < n; i++)
	{
		B.block(3 * i, 0, 3, 1) =
			Vn[i + m_numberOfFixR + 1] - Vn[i + m_numberOfFixR] - R0 * (V[i + m_numberOfFixR + 1] - V[i + m_numberOfFixR]);
	}
	//std::cout << "B:\n" << B << std::endl;
	Ve = (A.transpose() * A).inverse() * A.transpose() * B;
	//std::cout << "Ve:\n" << Ve << std::endl;
}

void RobotRegistration::calculateV0()
{
	V0.setZero();
	for (int i = 0; i < m_numberOfPose; i++)
	{
		V0 = V0 + (Vn[i] - R0 * R[i] * Ve - R0 * V[i]);
	}
	V0 = V0 / R.size();
	//std::cout << "V0:\n" << V0 << std::endl;
}

void RobotRegistration::genTestData()
{
	//Tm2b
	Transform<double, 3, Affine> transform = Translation3d(10, 10, 10) * AngleAxis<double>(M_PI / 6, Vector3d(0, 0, 1));
	Matrix4d Tm2b = transform.matrix();
	std::cout << "Tm2b:\n" << Tm2b << std::endl;
	//Tf2e TCP
	transform = Translation3d(5, 5, 5) * AngleAxis<double>(M_PI / 2, Vector3d(0, 1, 0));
	Matrix4d Tf2e = transform.matrix();
	std::cout << "Tf2e:\n" << Tf2e << std::endl;

	//10 pose of robot arm,first 5 fixR;
	std::vector<Matrix4d> Tb2f;
	//
	transform = Translation3d(5, 5, 5) * AngleAxis<double>(M_PI / 2, Vector3d(1, 2, 3).normalized());
	Tb2f.push_back(transform.matrix());
	transform = Translation3d(5, 3, 10) * AngleAxis<double>(M_PI / 2, Vector3d(1, 2, 3).normalized());
	Tb2f.push_back(transform.matrix());
	transform = Translation3d(5, 5, 4) * AngleAxis<double>(M_PI / 2, Vector3d(1, 2, 3).normalized());
	Tb2f.push_back(transform.matrix());
	transform = Translation3d(1, 5, 5) * AngleAxis<double>(M_PI / 2, Vector3d(1, 2, 3).normalized());
	Tb2f.push_back(transform.matrix());
	transform = Translation3d(5, 2, 6) * AngleAxis<double>(M_PI / 2, Vector3d(1, 2, 3).normalized());
	Tb2f.push_back(transform.matrix());
	//
	transform = Translation3d(1, 5, 7) * AngleAxis<double>(M_PI / 6, Vector3d(1, 2, 3).normalized());
	Tb2f.push_back(transform.matrix());
	transform = Translation3d(5, 3, 10) * AngleAxis<double>(M_PI / 2, Vector3d(2, 1, 3).normalized());
	Tb2f.push_back(transform.matrix());
	transform = Translation3d(2, 5, 4) * AngleAxis<double>(M_PI / 5, Vector3d(1, 2, 3).normalized());
	Tb2f.push_back(transform.matrix());
	transform = Translation3d(1, 8, 3) * AngleAxis<double>(M_PI / 9, Vector3d(1, 0, 0).normalized());
	Tb2f.push_back(transform.matrix());
	transform = Translation3d(5, 2, 6) * AngleAxis<double>(M_PI / 2, Vector3d(1, 2, 0).normalized());
	Tb2f.push_back(transform.matrix());

	std::vector<Matrix4d> Tm2e;
	for (auto t : Tb2f)
	{
		Tm2e.push_back(Tm2b * t * Tf2e);
	}

	for (int i = 0; i < 10; i++)
	{
		Matrix3d r = Tb2f[i].block(0, 0, 3, 3);
		Vector3d v = Tb2f[i].block(0, 3, 3, 1);

		Matrix3d rn = Tm2e[i].block(0, 0, 3, 3);
		Vector3d vn = Tm2e[i].block(0, 3, 3, 1);

		R.push_back(r);
		V.push_back(v);

		Rn.push_back(rn);
		Vn.push_back(vn);
	}

	m_numberOfFixR = 5;
}

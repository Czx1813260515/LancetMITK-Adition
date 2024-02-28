
#ifndef THAFEMURSTEMCOUPLE_H
#define THAFEMURSTEMCOUPLE_H

#include <mitkImageToImageFilter.h>

// The following header file is generated by CMake and thus it's located in
// the build directory. It provides an export macro for classes and functions
// that you want to be part of the public interface of your module.
#include <MitkLancetIGTExports.h>

#include "lancetThaStemObject.h"
#include "lancetThaFemurObject.h"
#include "mitkDataNode.h"
#include "mitkPointSet.h"
#include "mitkSurface.h"

namespace lancet
{
	/**Documentation
	  * \brief 
	  *
	  * \ingroup IGT
	  */
	class MITKLANCETIGT_EXPORT ThaFemurStemCouple : public itk::DataObject
	{
	public:
		mitkClassMacroItkParent(ThaFemurStemCouple, itk::DataObject);
		itkFactorylessNewMacro(Self)
		itkCloneMacro(Self)

		itkSetMacro(FemurObject, lancet::ThaFemurObject::Pointer)
		itkGetMacro(FemurObject, lancet::ThaFemurObject::Pointer)

		itkSetMacro(StemObject, lancet::ThaStemObject::Pointer)
		itkGetMacro(StemObject, lancet::ThaStemObject::Pointer)

		void SetCoupleGeometry(vtkSmartPointer<vtkMatrix4x4> newMatrix);
		itkGetMacro(vtkMatrix_coupleGeometry, vtkSmartPointer<vtkMatrix4x4>)

		void SetFemurFrameToStemFrameMatrix(vtkSmartPointer<vtkMatrix4x4> newMatrix);
		itkGetMacro(vtkMatrix_femurFrameToStemFrame, vtkSmartPointer<vtkMatrix4x4>)

		void InitializeFemurFrameToStemFrameMatrix();

		// Convenient method of getting the stem HeadCenter in femur frame
		mitk::Point3D GetStemHeadCenterInFemurFrame();

		// Calculate and return femurStemCoupleFrame --> femurCanalFrame
		vtkSmartPointer<vtkMatrix4x4> GetCoupleFrameToCanalFrame();

		vtkSmartPointer<vtkMatrix4x4> GetCoupleFrameToMechanicFrame();

	protected:
		ThaFemurStemCouple();
		ThaFemurStemCouple(const ThaFemurStemCouple& other);
		~ThaFemurStemCouple() override;

		lancet::ThaFemurObject::Pointer m_FemurObject;

		lancet::ThaStemObject::Pointer m_StemObject;

		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_femurFrameToStemFrame;

		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_coupleGeometry;

		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_coupleFrameToCanalFrame;

		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_coupleFrameToMechanicFrame;


	};
}

#endif

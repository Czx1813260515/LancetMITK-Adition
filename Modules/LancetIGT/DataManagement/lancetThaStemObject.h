
#ifndef THASTEMOBJECT_H
#define THASTEMOBJECT_H

#include <mitkImageToImageFilter.h>

// The following header file is generated by CMake and thus it's located in
// the build directory. It provides an export macro for classes and functions
// that you want to be part of the public interface of your module.
#include <MitkLancetIGTExports.h>

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
	class MITKLANCETIGT_EXPORT ThaStemObject : public itk::DataObject
	{
	public:
		mitkClassMacroItkParent(ThaStemObject, itk::DataObject);
		itkFactorylessNewMacro(Self)
		itkCloneMacro(Self)

		itkSetMacro(OperationSide, int)
		itkGetMacro(OperationSide, int)

		itkGetMacro(vtkMatrix_groupGeometry, vtkSmartPointer<vtkMatrix4x4>)
		/*
		 * Update the geometry matrix of each data component (do not rewrite the data)
		 * with newMatrix and m_vtkMatrix_groupGeometry;
		 * replace m_vtkMatrix_groupGeometry with newMatrix
		 */
		void SetGroupGeometry(vtkSmartPointer<vtkMatrix4x4> newMatrix);

		itkGetMacro(Surface_stem, mitk::Surface::Pointer)
		// set stem surface and apply the group geometry
		void SetStemSurface(mitk::Surface::Pointer stemSurface);

		itkSetMacro(Surface_stemFrame, mitk::Surface::Pointer)
		itkGetMacro(Surface_stemFrame, mitk::Surface::Pointer)

		itkGetMacro(Pset_headCenter, mitk::PointSet::Pointer)
		// set headCenter and apply the group geometry, and move the head surface
		void SetHeadCenter(mitk::PointSet::Pointer headCenterPset_stemFrame_new);

		itkGetMacro(Surface_head, mitk::Surface::Pointer);
		// set head surface and apply the group geometry considering the headCenter
		void SetHeadSurface(mitk::Surface::Pointer headSurface);

		/*
		 * Move all the data components except for the stemFrame
		 * which is created at the world frame position,
		 * (rewrite the data, keep the geometry matrices as identity)
		 * so that the stem frame overlays with the world frame
		 */
		bool AlignStemObjectWithWorldFrame();

		// Check if all the necessary data are ready 
		bool CheckDataAvailability();

	protected:

		ThaStemObject();
		ThaStemObject(const ThaStemObject& other);
		~ThaStemObject();
		
		// Create an internal frame
		void CreateInternalFrame();

		// operation side: right(0), left(1)
		int m_OperationSide{ 0 };

		// the geometry matrix of the pelvis group object
		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_groupGeometry;

		// femoral stem surface
		mitk::Surface::Pointer m_Surface_stem;

		// head surface
		mitk::Surface::Pointer m_Surface_head;
		
		// rotation center
		mitk::PointSet::Pointer m_Pset_headCenter;

		// stem frame surface
		mitk::Surface::Pointer m_Surface_stemFrame;

	};
}

#endif

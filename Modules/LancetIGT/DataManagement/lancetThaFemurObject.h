/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef THAFEMUROBJECT_H
#define THAFEMUROBJECT_H

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
	class MITKLANCETIGT_EXPORT ThaFemurObject : public itk::DataObject
	{
	public:
		mitkClassMacroItkParent(ThaFemurObject, itk::DataObject);
		itkFactorylessNewMacro(Self)
		itkCloneMacro(Self)

		itkGetMacro(femurSide, int)
		itkSetMacro(femurSide, int)

		itkGetMacro(isOperationSide, int)
		itkSetMacro(isOperationSide, int)

		itkGetMacro(femurVersion, double)
		itkSetMacro(femurVersion, double)

		itkGetMacro(image_femur, mitk::Image::Pointer)
		itkSetMacro(image_femur, mitk::Image::Pointer)

		itkGetMacro(surface_femur, mitk::Surface::Pointer)
		itkSetMacro(surface_femur, mitk::Surface::Pointer)

		itkGetMacro(surface_femurFrame, mitk::Surface::Pointer)
		itkSetMacro(surface_femurFrame, mitk::Surface::Pointer)

		itkSetMacro(pset_neckCenter, mitk::PointSet::Pointer)
		itkGetMacro(pset_neckCenter, mitk::PointSet::Pointer)

		itkSetMacro(pset_lesserTrochanter, mitk::PointSet::Pointer)
		itkGetMacro(pset_lesserTrochanter, mitk::PointSet::Pointer)

		itkSetMacro(pset_femurCOR, mitk::PointSet::Pointer)
		itkGetMacro(pset_femurCOR, mitk::PointSet::Pointer)

		itkSetMacro(pset_femurCanal, mitk::PointSet::Pointer)
		itkGetMacro(pset_femurCanal, mitk::PointSet::Pointer)

		itkSetMacro(pset_epicondyles, mitk::PointSet::Pointer)
		itkGetMacro(pset_epicondyles, mitk::PointSet::Pointer)

		itkGetMacro(vtkMatrix_groupGeometry, vtkSmartPointer<vtkMatrix4x4>)

		itkGetMacro(vtkMatrix_canalFrameToMechanicFrame, vtkSmartPointer<vtkMatrix4x4>)

		/*
		 * Update the geometry matrix of each data component (do not rewrite the data)
		 * with newMatrix and m_vtkMatrix_groupGeometry (be careful with mitk::Image);
		 * replace m_vtkMatrix_groupGeometry with newMatrix
		 */
		void SetGroupGeometry(vtkSmartPointer<vtkMatrix4x4> newMatrix);

		/*
		 * Move all the data components except for the pelvisFrame
		 * which is created at the world frame position,
		 * (rewrite the data, keep the geometry matrices as identity)
		 * so that the femur frame overlays with the world frame
		 */
		bool AlignFemurObjectWithWorldFrame();

		// Check if all the necessary data are ready
		// should consider m_isOperationSide
		// if m_isOperationSide == 1, more data are needed
		bool CheckDataAvailability();

	protected:

		ThaFemurObject();
		ThaFemurObject(const ThaFemurObject& other);
		~ThaFemurObject() override;

		// Append the geometry matrix to the pointSet
		void RewritePointSetWithGeometryMatrix(mitk::PointSet::Pointer inputPointSet);
		mitk::PointSet::Pointer GetPointSetWithGeometryMatrix(const mitk::PointSet::Pointer inputPointSet);

		// Create an internal frame
		void CreateInternalFrame();

		/*
		 * Establish a femur coordinate with the proximal femurCanal point as the origin O,
		 * femurCanal[1] --> femurCanal[0] as the z direction
		 * z X (O --> femurCOR) as the y direction for right femur
		 * (O --> femurCOR) X z as the y direction for left femur
		 * return the transform matrix from the world frame to the femur frame
		 */
		vtkSmartPointer<vtkMatrix4x4> CalculateWorldToFemurTransform();

		// Calculate m_vtkMatrix_canalFrameToMechanicFrame
		void CalculateCanalFrameToMechanicFrame();

		void CalculateFemurVersion();

		// the geometry matrix of the femur group object
		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_groupGeometry;

		// transform matrix from the femurCanal frame (femurFrame) to the mechanical axis frame
		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_canalFrameToMechanicFrame;

		// right femur is 0; left femur is 1
		int m_femurSide{ 0 };

		// 0 if this femur is not at the operation side; 1 if at the operation side
		int m_isOperationSide{ 0 };

		// anteversion is (+), retroversion is (-)
		double m_femurVersion{ 0 };

		// femur image
		mitk::Image::Pointer m_image_femur;

		// femur surface
		mitk::Surface::Pointer m_surface_femur;

		// femurFrame surface
		mitk::Surface::Pointer m_surface_femurFrame;

		// neck center
		mitk::PointSet::Pointer m_pset_neckCenter;

		// lesser trochanter
		mitk::PointSet::Pointer m_pset_lesserTrochanter;

		// femur head center (center of rotation)
		mitk::PointSet::Pointer m_pset_femurCOR;

		// femur canal (anatomical axis); point 0 is proximal; point 1 is distal
		mitk::PointSet::Pointer m_pset_femurCanal;

		// femur epicondyles; 0 is medial; 1 is lateral
		mitk::PointSet::Pointer m_pset_epicondyles;

	};
}

#endif

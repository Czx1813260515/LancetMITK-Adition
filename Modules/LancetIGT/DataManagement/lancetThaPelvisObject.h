/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef THAPELVISOBJECT_H
#define THAPELVISOBJECT_H

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
	class MITKLANCETIGT_EXPORT ThaPelvisObject : public itk::DataObject
	{
	public:
		mitkClassMacroItkParent(ThaPelvisObject, itk::DataObject);
		itkFactorylessNewMacro(Self)
		itkCloneMacro(Self)

		itkGetMacro(pelvicTilt_supine, double)
		itkSetMacro(pelvicTilt_supine,double)

		itkGetMacro(pelvicTilt_stand, double)
		itkSetMacro(pelvicTilt_stand, double)

		itkGetMacro(pelvicTilt_sit, double)
		itkSetMacro(pelvicTilt_sit, double)

		// itkSetMacro(vtkMatrix_groupGeometry, vtkSmartPointer<vtkMatrix4x4>)
		itkGetMacro(vtkMatrix_groupGeometry, vtkSmartPointer<vtkMatrix4x4>)

		itkSetMacro(surface_pelvis, mitk::Surface::Pointer)
		itkGetMacro(surface_pelvis, mitk::Surface::Pointer)

		itkSetMacro(surface_pelvisFrame, mitk::Surface::Pointer)
		itkGetMacro(surface_pelvisFrame, mitk::Surface::Pointer)

		itkSetMacro(pset_ASIS, mitk::PointSet::Pointer)
		itkGetMacro(pset_ASIS, mitk::PointSet::Pointer)

		itkSetMacro(pset_pelvisCOR, mitk::PointSet::Pointer)
		itkGetMacro(pset_pelvisCOR, mitk::PointSet::Pointer)

		itkSetMacro(pset_midline, mitk::PointSet::Pointer)
		itkGetMacro(pset_midline, mitk::PointSet::Pointer)

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
		 * so that the pelvis frame overlays with the world frame
		 */
		bool AlignPelvicObjectWithWorldFrame();

		// Check if all the necessary data are ready 
		bool CheckDataAvailability();

	protected:

		ThaPelvisObject();
		ThaPelvisObject(const ThaPelvisObject& other);
		~ThaPelvisObject() override;

		// Append the geometry matrix to the pointSet
		void RewritePointSetWithGeometryMatrix( mitk::PointSet::Pointer inputPointSet);
		mitk::PointSet::Pointer GetPointSetWithGeometryMatrix(const mitk::PointSet::Pointer inputPointSet);

		/*
		 * Establish a pelvis coordinate with the midline as the origin O,
		 * ASIS[0] --> ASIS[1] as the x direction
		 * (O --> ASIS[0]) X (O --> ASIS[1]) as the y direction,
		 * return the transform matrix from the world frame to the pelvis frame
		 */
		vtkSmartPointer<vtkMatrix4x4> CalculateWorldToPelvisTransform();

		/* Calculate m_pelvicTilt_supine
		 * must ensure the patient direction in the CT image
		 * from which the pelvis surface is reconstructed meets the following requirements:
		 * feet --> head is approximately the CT z axis
		 * right side --> left side is approximately the CT x axis
		 * anterior --> posterior is approximately the CT y axis;
		 * otherwise, the pelvic tilt might be wrong
		 */
		void CalculateSupinePelvicTilt();

		// Create an internal frame
		void CreateInternalFrame();

		// the geometry matrix of the pelvis group object
		vtkSmartPointer<vtkMatrix4x4> m_vtkMatrix_groupGeometry;

		// supine pelvic tilt in degree, anterior(+), posterior(-)
		double m_pelvicTilt_supine{ 0 };

		// stand pelvic tilt in degree, anterior(+), posterior(-)
		double m_pelvicTilt_stand{ 0 };

		// sit pelvic tilt in degree, anterior(+), posterior(-)
		double m_pelvicTilt_sit{ 0 };

		// the complete pelvis surface
		mitk::Surface::Pointer m_surface_pelvis;
		
		// the pelvis coordinate
		mitk::Surface::Pointer m_surface_pelvisFrame;
		
		// ASIS axis, point 0 is the right side, point 1 is the left side
		mitk::PointSet::Pointer m_pset_ASIS;
		
		// acetabular center of rotation, point 0 is the right side, point 1 is the left side
		mitk::PointSet::Pointer m_pset_pelvisCOR;
		
		// the pelvis midline point, contains one point
		mitk::PointSet::Pointer m_pset_midline;
		

	};
}

#endif

/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef LANCETAPPLYDEVICEREGISTRATIOINFILTER_H
#define LANCETAPPLYDEVICEREGISTRATIOINFILTER_H
#include "mitkNavigationDataToNavigationDataFilter.h"
#include "MitkLancetIGTExports.h"

namespace lancet {

  /**Documentation
  * \brief NavigationDataReferenceTransformFilter applies a itk-landmark-transformation
  * defined by source and target NavigationDatas.
  *
  * Before executing the filter SetSourceNavigationDatas and SetTargetNavigationDatas must be called.
  * The amount of given NavigationDatas must be the same for source and target.
  * If source or target points are changed after calling SetXXXNavigationDatas, the corresponding SetXXXNavigationDatas
  * method has to be called again to apply the changes.
  * If UseICPInitialization is false (standard value, or set with SetUseICPInitialization(false) or UseICPInitializationOff())
  * then source landmarks and target landmarks with the same ID must correspondent to each other.
  * (--> source landmark with ID x will be mapped to target landmark with ID x).
  * If you do not know the correspondences, call SetUseICPInitialization(true) or UseICPInitializationOn()
  * to let the filter guess the correspondences during initialization with an iterative closest point search.
  * This is only possible, if at least 6 source and target landmarks are available.
  *
  * \ingroup IGT
  */
  class MITKLANCETIGT_EXPORT ApplyDeviceRegistratioinFilter : public mitk::NavigationDataToNavigationDataFilter
  {
  public:
    mitkClassMacro(ApplyDeviceRegistratioinFilter, NavigationDataToNavigationDataFilter);
    itkFactorylessNewMacro(Self);
    itkCloneMacro(Self);

    itkGetMacro(RefToolIndex, unsigned int);
    itkSetMacro(RefToolIndex, unsigned int);


  protected:

    /**
    * \brief Constructor
    **/
    ApplyDeviceRegistratioinFilter();
    /**
    * \brief Destructor
    **/
    ~ApplyDeviceRegistratioinFilter() override;

    void GenerateData() override;

    static mitk::AffineTransform3D::Pointer NavigationDataToTransform(
      const mitk::NavigationData *nd);

    static vtkMatrix4x4* NavigationDataToVtkMatrix4x4(const mitk::NavigationData *nd);

    //static mitk::NavigationData::Pointer vtkMatrix4x4ToNavigationData(vtkMatrix4x4* matrix4x4);

    static mitk::Point3D vtkMatrix4x4ToPosition(vtkMatrix4x4 *matrix4x4);

    static mitk::NavigationData::OrientationType vtkMatrix4x4ToQuaternion(vtkMatrix4x4 *matrix4x4);

    void vtkMatrix4x4ToMatrix3x3(const vtkMatrix4x4* input, double A[3][3]);

    static void GetReferenceMatrix4x4(vtkMatrix4x4 *in_matrix,
                                      vtkMatrix4x4 *ref_matrix,
                                      vtkMatrix4x4 *res_matrix);

    unsigned int m_RefToolIndex{0};


  };

  inline void ApplyDeviceRegistratioinFilter::GetReferenceMatrix4x4(vtkMatrix4x4 *in_matrix,
    vtkMatrix4x4 *ref_matrix,
    vtkMatrix4x4 *res_matrix)
  {
    vtkSmartPointer<vtkMatrix4x4> ref_matrix_invert = vtkMatrix4x4::New();
    ref_matrix_invert->DeepCopy(ref_matrix);
    ref_matrix_invert->Invert();
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->PostMultiply();
    transform->SetMatrix(in_matrix);
    transform->Concatenate(ref_matrix_invert);
    transform->Update();
    transform->GetMatrix(res_matrix);
  }
} // namespace lancet

#endif // LANCETAPPLYDEVICEREGISTRATIOINFILTER_H

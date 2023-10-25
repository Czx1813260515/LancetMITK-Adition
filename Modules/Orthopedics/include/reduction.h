#ifndef reduction_h
#define reduction_h

#include <MitkOrthopedicsExports.h>
#include "body.h"

namespace othopedics
{
	class MITKORTHOPEDICS_EXPORT Reduction :public itk::Object
	{
	public:
		mitkClassMacroItkParent(Reduction, itk::Object);
		itkNewMacro(Reduction);

		itkSetMacro(Pelvis, Pelvis::Pointer);
		itkSetMacro(Femur_L, Femur::Pointer);
		itkSetMacro(Femur_R, Femur::Pointer);

		itkGetMacro(Pelvis, Pelvis::Pointer);
		itkGetMacro(Femur_L, Femur::Pointer);
		itkGetMacro(Femur_R, Femur::Pointer);

		virtual void SetResult(EResult name, double res);
		virtual bool GetResult(EResult name, double& outp_res);

		void PreOperativeReduction_Canal();
		void PreOperativeReduction_Mechanical();
		void CalPreopOffset();
		void CalPreopHipLength();
		ESide m_OperationSide{ESide::right};
	private:
		Pelvis::Pointer m_Pelvis;
		Femur::Pointer m_Femur_L;
		Femur::Pointer m_Femur_R;

		std::map<EResult, double> m_results{};
	};
}



#endif

/**
 * \par Copyright (C)2022-2023 Hangzhou Lancet Robot Co., Ltd. All rights reserved.
 *
 * \brief $Document description$
 * \ingroup org_mitk_lancet_services_medicalrecordsadministration
 * \version V1.0.0
 * \data 2022-10-11 10:45:02
 * 
 * \par Modify History
 *
 * \author Sun
 * \data 2022-10-11 10:45:02
 * \remark $Modify Description$
 */
#ifndef LancetIMedicalRecordsProperty_H
#define LancetIMedicalRecordsProperty_H

// c or c++
#include <memory>

// berry
#include <berryObject.h>

// Qt
#include <QSharedPointer>

// ORG_MITK_LANCET_SERVICES_MEDICALRECORDSADMINISTRATION_PLUGIN
#include "org_mitk_lancet_services_medicalrecordsadministration_Export.h"

class QString;
class QVariant;
namespace lancet
{
/**
 * \class IMedicalRecordsProperty
 * \ingroup org_mitk_lancet_services_medicalrecordsadministration
 * \namespace lancet
 * \brief 
 *
 * TODO: long description
 *
 * \pre $Pre description information$
 * \example $Description of use examples$
 * \code
 *	todo...
 * \endcoed
 *
 * \author Sun
 * \version V1.0.0
 * \date 2022-10-11 10:44:44
 * \remark todo: insert comments
 *
 * Contact: sh4a01@163.com
 *
 */
class ORG_MITK_LANCET_SERVICES_MEDICALRECORDSADMINISTRATION_PLUGIN
	IMedicalRecordsProperty : public QObject, public berry::Object
{
	Q_OBJECT
public:
	berryObjectMacro(lancet::IMedicalRecordsProperty);
	IMedicalRecordsProperty();

	/**
	 * \brief
	 */
	struct ORG_MITK_LANCET_SERVICES_MEDICALRECORDSADMINISTRATION_PLUGIN
		PropertyKeys
	{
		using PropertyKey = const char* const;

		static PropertyKey Key;			///> Ψһֵ����{[0-9][a-z][A-Z]}��ɵ������ַ���
		static PropertyKey Name;		///> ��������
		static PropertyKey Id;			///> ����ID
		static PropertyKey Sex;			///> �Ա�
		static PropertyKey Age;			///> ����
		static PropertyKey Valid;		///> �Ƿ���Ч��False��Чϵͳ��ʶ��
		static PropertyKey OperatingSurgeonName;	///> ����ҽ������
		static PropertyKey SurgicalArea;					///> ��������
		static PropertyKey Type;		///> ������Ʒ������
		static PropertyKey CreateTime;			///> ����ʱ��
		static PropertyKey LastModifyTime;	///> ����޸�ʱ��

		static PropertyKey DataAddress;			///> ���ݵ�ַ
		static PropertyKey DrPreviewAddress;			///> DRԤ�����ݵ�ַ
		static PropertyKey DrPositiveDicomAddress;	///> DR��λƬDicom�ļ���ַ
		static PropertyKey DrLateralDicomAddress;		///> DR��λƬDicom�ļ���ַ

		static QList<QString> ToQtList();
	};
public:
	virtual bool HasKey(const QString&) const;
	virtual bool HasModifyOf(const QString&) const;
	virtual bool HasModifyOf(const QList<QString>&) const;

	virtual void ModifyOf(const IMedicalRecordsProperty*);
	virtual void ModifyOf(const IMedicalRecordsProperty::Pointer&);

	virtual QList<QString> GetKeys() const;
	virtual QList<QString> GetModifyKeys() const;

	virtual QVariant GetKeyValue(const QString&) const;
	virtual void SetKeyValue(const QString&, const QVariant&);
	
	virtual void ResetPropertyOfModify();
public:
	virtual void operator=(const IMedicalRecordsProperty&);
	virtual void operator=(IMedicalRecordsProperty*);
	virtual void operator=(IMedicalRecordsProperty::Pointer);

	virtual bool operator==(const IMedicalRecordsProperty&) = 0;
	virtual bool operator==(IMedicalRecordsProperty*);
	virtual bool operator==(IMedicalRecordsProperty::Pointer);
public:
	virtual void initializeKeys() = 0;
	virtual QString ToString() const = 0;
	virtual QString ToJsonString() const = 0;
private:
	struct IMedicalRecordsPropertyPrivateImp;
	std::shared_ptr<IMedicalRecordsPropertyPrivateImp> imp;
};
}
using QPropertyKeys = lancet::IMedicalRecordsProperty::PropertyKeys;
using QMedicalRecordsProperty = QSharedPointer<lancet::IMedicalRecordsProperty>;
using QMedicalRecordsPropertyArray = QVector<QMedicalRecordsProperty>;

#endif // !LancetIMedicalRecordsProperty_H

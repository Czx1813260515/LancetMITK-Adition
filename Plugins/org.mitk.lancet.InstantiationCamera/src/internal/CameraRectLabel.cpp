#include "CameraRectLabel.h"


CameraRectLabel::CameraRectLabel(QWidget* parent) : QLabel(parent)
{
	setMouseTracking(true);
}

void CameraRectLabel::StartDraw()
{
	m_IsSelecting = true;
	std::cout << "StartDraw m_IsSelecting is true" << std::endl; 
	update();
}

void CameraRectLabel::RequestPaint()
{
	//if (!m_CurrentRect.isNull()) {
	//	QPainter painter(this);
	//	painter.setPen(QPen(Qt::red, 2));  // ʹ�ú�ɫ���ʻ��ƾ��ο�
	//	painter.drawRect(m_CurrentRect);     // ���ƾ��ο�
	//}
}

void CameraRectLabel::mousePressEvent(QMouseEvent* event)
{
	if (m_IsSelecting && event->button() == Qt::LeftButton) {
		m_SelectionStart = true;     // ��ǿ�ѡ�ѿ�ʼ
		m_StartPoint = event->pos();   // ��¼��ʼ��
		m_CurrentRect = QRect();       // ����ϴεľ���
		std::cout << "mousePressEvent" << std::endl;
	}
}

void CameraRectLabel::mouseMoveEvent(QMouseEvent* event)
{
	if (m_IsSelecting && m_SelectionStart) {
		// ���µ�ǰ���ο�Ĵ�С
		std::cout << "mouseMoveEvent" << std::endl;
		m_CurrentRect = QRect(m_StartPoint, event->pos()).normalized();
		update();  // ˢ�½��棬���� paintEvent �ػ�
	}
}

void CameraRectLabel::mouseReleaseEvent(QMouseEvent* event)
{
	if (m_IsSelecting && m_SelectionStart && event->button() == Qt::LeftButton) {
		m_SelectionStart = false;  // ��ѡ����
		m_IsSelecting = false;         // �˳���ѡģʽ
		update();                  // ˢ�½�����ʾ���վ���
		std::cout << "mouseReleaseEvent" << std::endl;
	}
}

void CameraRectLabel::paintEvent(QPaintEvent* event)
{
	QLabel::paintEvent(event);
	//std::cout << "paintEvent" << std::endl;
	if (!m_CurrentRect.isNull()) {
		QPainter painter(this);
		painter.setPen(QPen(Qt::red, 2));  // ʹ�ú�ɫ���ʻ��ƾ��ο�
		painter.drawRect(m_CurrentRect);     // ���ƾ��ο�
	
	}
}

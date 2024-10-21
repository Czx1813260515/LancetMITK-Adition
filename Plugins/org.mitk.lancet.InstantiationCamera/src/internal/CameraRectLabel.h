#pragma once
#include <qlabel.h>
#include <qevent.h>
#include <qpainter.h>
class CameraRectLabel : public QLabel
{
	Q_OBJECT
public:
	explicit CameraRectLabel(QLabel* aLabel = nullptr);

	void StartDraw();
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;

	void mouseReleaseEvent(QMouseEvent* event) override;

	void paintEvent(QPaintEvent* event) override;

private:
	QLabel* m_Label;
	bool m_IsSelecting = false;
	QPoint m_StartPoint;  // ��ѡ����ʼ��
	QPoint m_EndPoint;    // ��ѡ�Ľ�����
};


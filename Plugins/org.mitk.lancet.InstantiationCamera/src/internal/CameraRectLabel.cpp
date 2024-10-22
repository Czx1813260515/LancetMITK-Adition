#include "CameraRectLabel.h"

CameraRectLabel::CameraRectLabel(QLabel* aLabel)
{
    m_Label = aLabel;
}

void CameraRectLabel::StartDraw()
{
    m_IsSelecting = true;
}

void CameraRectLabel::mousePressEvent(QMouseEvent* event)
{
    if (m_IsSelecting == true)
    {
        if (event->button() == Qt::LeftButton) {

            m_StartPoint = event->pos();
            m_EndPoint = m_StartPoint;  // ��ʼʱ�������������ͬ
            update();  // �����Դ����ػ�
        }
    }
}

void CameraRectLabel::mouseMoveEvent(QMouseEvent* event)
{
    if (m_IsSelecting) {
        m_EndPoint = event->pos();  // ���½�����Ϊ��ǰ���λ��
        update();  // �����Դ����ػ�
    }
}

void CameraRectLabel::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        m_IsSelecting = false;  // ����ѡ��
        update();  // ���һ���ػ�
    }
}

void CameraRectLabel::paintEvent(QPaintEvent* event)
{
    QLabel::paintEvent(event);

    if (m_IsSelecting) {
        QPainter painter(this);
        painter.setPen(QPen(Qt::red, 2, Qt::DashLine));  // ��ɫ���߿�
        QRect rect = QRect(m_StartPoint, m_EndPoint).normalized();  // ��һ������
        painter.drawRect(rect);  // ���ƾ���
    }
}

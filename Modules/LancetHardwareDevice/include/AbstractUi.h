// BaseUi.h
#ifndef BASEUI_H
#define BASEUI_H

#include <QWidget>

class BaseUi {
public:
    virtual ~BaseUi() {}
    virtual void setupUi(QWidget* widget) = 0; // ���麯��
};

#endif // BASEUI_H

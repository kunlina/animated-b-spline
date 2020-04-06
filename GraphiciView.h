#ifndef GRAPHICIVIEW_H
#define GRAPHICIVIEW_H

#include <QGraphicsView>

class GraphiciView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit GraphiciView(QWidget *parent = nullptr);

private:
    void wheelEvent(QWheelEvent *event);

    int m_scale;
};

#endif // GRAPHICIVIEW_H

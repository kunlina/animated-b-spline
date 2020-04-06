#include "GraphiciView.h"

#include <QWheelEvent>
#include <QScrollBar>

GraphiciView::GraphiciView(QWidget *parent)
    : QGraphicsView(parent)
{
    setDragMode(QGraphicsView::ScrollHandDrag);//(QGraphicsView::RubberBandDrag);//QGraphicsView::ScrollHandDrag
    m_scale = 1;//图形原始比例
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
}

void GraphiciView::wheelEvent(QWheelEvent *event)
{
    //按住ctrl键 可以放大缩小
    if (event->modifiers() == Qt::CTRL) {
        //最大放大到原始图像的50倍
        if((event->delta() > 0)&&(m_scale >= 50)) {
            return;
        } else if((event->delta() < 0)&&(m_scale <= 0.01))  { //图像缩小到自适应大小之后就不继续缩小
            return;//重置图片大小和位置，使之自适应控件窗口大小
        } else {
            // 当前放缩倍数;
            qreal scaleFactor = this->matrix().m11();
            m_scale = scaleFactor;

            int wheelDeltaValue = event->delta();
            if (wheelDeltaValue > 0) { // 向上滚动，放大;
                this->scale(1.2, 1.2);
            } else { // 向下滚动，缩小;
                this->scale(1.0 / 1.2, 1.0 / 1.2);
            }
            update();
            event->accept();
        }
    } else {
        QGraphicsView::wheelEvent(event);
        update();
    }
}





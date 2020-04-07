#include "movingellipseitem.h"
#include <QGraphicsSceneMouseEvent>
#include <QDebug>

MovingEllipseItem::MovingEllipseItem(QRectF f, MainWindow *mainWindow, QGraphicsItem *parent)
    : QGraphicsEllipseItem(f, parent)
    , mainWindow(mainWindow)
{
}

MovingEllipseItem::MovingEllipseItem(qreal x, qreal y, qreal width,
                                     qreal height, MainWindow *mainWindow,
                                     QGraphicsItem *parent)
    : QGraphicsEllipseItem(x, y, width, height, parent)
    , mainWindow(mainWindow)
{}


// mouseMoveEvent - moves control point on QGraphicsView, updates control
// point position in \var mainWindow->controlPoints and rerenders scene.
void MovingEllipseItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsEllipseItem::mouseMoveEvent(event);

    const QPointF pos = event->scenePos();
    mainWindow->ctrlPnts.at(mId).x = pos.x();
    mainWindow->ctrlPnts.at(mId).y = pos.y();

    // remove other items
    const QList<QGraphicsItem*> &items = mainWindow->scene->items();
    for (QList<QGraphicsItem*>::const_iterator itemIt = items.begin();
         itemIt != items.end(); ++itemIt) {

        if (*itemIt != this) {
            mainWindow->scene->removeItem(*itemIt);
            delete *itemIt;
        }
    }

    mainWindow->finalInterpolateCurve();
    mainWindow->update2DView(mId);
    mainWindow->update3DWidget();
}

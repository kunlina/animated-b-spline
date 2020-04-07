#ifndef MOVINGELLIPSEITEM_H
#define MOVINGELLIPSEITEM_H

#include "mainwindow.h"
#include <QGraphicsEllipseItem>
#include <QHash>

/// MovingEllipseItem - this class represents control point on QGraphicsView on
/// \var mainWindow.
class MovingEllipseItem : public QGraphicsEllipseItem {
public:
    explicit MovingEllipseItem(QRectF f, MainWindow *mainWindow, QGraphicsItem *parent = Q_NULLPTR);
    MovingEllipseItem(qreal x, qreal y, qreal width, qreal height,
                      MainWindow *mainWindow, QGraphicsItem *parent = Q_NULLPTR);
    void setCtrlPntId(int id) {mId = id;}

private:
    MainWindow *mainWindow;
    int mId;

    /// mouseMoveEvent - moves control point on QGraphicsView, updates control
    /// point position in \var mainWindow->controlPoints and rerenders scene.
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

};

#endif // MOVINGELLIPSEITEM_H

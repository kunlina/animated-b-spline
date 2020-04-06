#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QHash>
#include <QTimer>
#include <bezier/bezierinterpolator.h>
#include <geometry/GeometryCompute.h>


class MovingEllipseItem;
class QSystemTrayIcon;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = Q_NULLPTR);
    ~MainWindow();

private slots:
    // display settings
    void onInterpPnt_stateChanged(int state);
    void onInterpLine_stateChanged(int state);
    void onControlPnt_stateChanged(int state);
    void onControlLine_stateChanged(int state);
    void onBoorPnt_stateChanged(int state);
    void onBoorLine_stateChanged(int state);
    void onEasyBezierPnt_stateChanged(int state);
    void onEasyBezierLine_stateChanged(int state);

    // curve parameters
    void onDegree_valueChanged(int value);
    void onPointNum_valueChanged(int value);

    // torlance settings
    void onTorence_valueChanged(double value);

    // animation setting
    void on_startStopButton_clicked();
    void on_RandomButton_clicked();
//    void on_AddPointButton_clicked();
//    void on_DelPointButton_clicked();

    // performance setting
    void on_AntialiasingSlider_sliderMoved(int position);
    void on_AntialiasingSlider_valueChanged(int value);
    void on_SpeedSlider_sliderMoved(int position);
    void on_SpeedSlider_valueChanged(int value);
    void on_horizontalSlider_sliderMoved(int position);
    void on_horizontalSlider_valueChanged(int value);
    void on_controlPtSlider_sliderMoved(int position);
    void onControlPtSlider_valueChanged(int position);

    void updateView(QPointF *skipPoint = 0);

    void moveCurve();

    void updateFPS();

    void about();
    void show3DScene();
    void show2DScene();
private:
    void clearSceneAndUpdateView();
    void createSystemTrayActions();
    // Crunch. After moving of control point scene must be rerendered and I am too
    // lazy to create public function for it.
    friend class MovingEllipseItem;

    // User interface.
    Ui::MainWindow *ui;

    // Main scene for spline visualization.
    QGraphicsScene *scene;

    QTimer *animationTimer; // Control points will be moved by this timer.
    QTimer *fpsTimer; // On this timer fpsLabel is updated.
    unsigned framesNumber; // Number of frames rendered so far.

    // Speed of control point is multiplied by this value before moving.
    double speedMultiplicator;

    QVector<QPointF*> controlPoints;
    QVector<qreal> knotVector;
    QPolygonF boorNetPoints;
    QPolygonF interpolatedPoints;

    // Mapping of items on the scene to points in \var controlPoints.
    QHash<MovingEllipseItem*, QPointF*> itemToPoint;

    QVector<QPointF> controlPointsSpeed;

    // Object with interface to boor net calculator and Bezier interpolation.
    BezierInterpolator bezierInterpolator;
    GeometryCompute easybezierInterpolator;
    QPolygonF easyBezierInterpolatedPoints;

    Nurbs mNurbsCurve;

    /// showRandomSpline - generate random control points and show them.
    void showRandomSpline();

    /// interpolateCurve - calculate new control points with de Boor algorithm,
    /// break curve into multiple Bezier curves and interpolate each Bezier curve.
    void interpolateCurve();

    /// fillKnotVector - fill \var knotVector with knots for uniform cubic
    /// B-spline that passes through endpoints.
    void fillKnotVector();

    void showKnotVector();

    /// addControlPoint - adds control point within borders of \var graphicsView.
    void addControlPoint();
    /// clearPoints - delete all control points properly.
    void clearPoints();

    void showControlPoints();

    void updateStatusBar();

    struct DisplaySettings {
        DisplaySettings() : showBezierPoints(false), showBezierLine(true), showControlPoints(true),
            showBoorPoints(false), showControlLines(true), showBoorLines(false),
            showEasyBezierLine(true), showEasyBezierInterpolatedPoints(true), controlPointSize(10)
        {}

        bool showBezierPoints;
        bool showBezierLine;
        bool showControlPoints;
        bool showControlLines;
        bool showBoorPoints;
        bool showBoorLines;
        bool showEasyBezierLine;
        bool showEasyBezierInterpolatedPoints;
        int controlPointSize;
    } displaySettings;

    QAction *restoreAction;
    QAction *quitAction;

    QSystemTrayIcon *trayIcon;
    QMenu *trayIconMenu;
    void createTrayIcon();
    void createActions();
    void closeEvent(QCloseEvent *event);
    void createMenus();
    void createToolBars();
    void easyInterpolateCurve();
    void finalInterpolateCurve();
};

#endif // MAINWINDOW_H

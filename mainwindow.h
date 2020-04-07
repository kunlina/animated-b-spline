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

namespace  Qwt3D{
class CurvePlot;
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


    void moveCurve();

    void updateFPS();

    void about();
    void show3DScene();
    void show2DScene();
private:
    void clearAndUpdateAllView();
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

    std::vector<Point> ctrlPnts;
    QVector<qreal> knotVector;
    QPolygonF boorNetPoints;
    QPolygonF interpolatedPoints;

    QVector<QPointF> controlPointsSpeed;

    // Object with interface to boor net calculator and Bezier interpolation.
    BezierInterpolator bezierInterpolator;
    GeometryCompute easybezierInterpolator;
    std::vector<Point> subdiviedPnts;

    Nurbs mNurbsCurve;

    /// showRandomSpline - generate random control points and show them.
    void showRandomSpline();

#if 0
    /// interpolateCurve - calculate new control points with de Boor algorithm,
    /// break curve into multiple Bezier curves and interpolate each Bezier curve.
    void interpolateCurve();
#endif

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
            easyBezierLine(true), easyBezierInterpolatedPnts(true), controlPointSize(10)
        {}

        bool showBezierPoints;
        bool showBezierLine;
        bool showControlPoints;
        bool showControlLines;
        bool showBoorPoints;
        bool showBoorLines;
        bool easyBezierLine;
        bool easyBezierInterpolatedPnts;
        int controlPointSize;
    } dspSettings;

    QAction *restoreAction;
    QAction *quitAction;

    QSystemTrayIcon *trayIcon;
    QMenu *trayIconMenu;
    Qwt3D::CurvePlot *m3DPlotWidet;
    void createTrayIcon();
    void createActions();
    void closeEvent(QCloseEvent *event);
    void createMenus();
    void createToolBars();
#if 0 // delete later
    void easyInterpolateCurve();
#endif
    void finalInterpolateCurve();
    void update3DWidget();
    void update2DView(int skipId = -1);
};

#endif // MAINWINDOW_H

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "movingellipseitem.h"
#include <QTime>

#include <QMenu>
#include <QAction>
#include <QSystemTrayIcon>
#include <QCloseEvent>
#include <QMessageBox>
#include <qwt3d/qwt3d_curveplot.h>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , framesNumber(0)
    , speedMultiplicator(1.0)
    , pointsNumber(5)
{
    // init property
    displaySettings.showInterpolatedPoints = true;
    fillKnotVector();

    // creat ui.
    ui->setupUi(this);
    scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    delete ui->scene3D;
    Qwt3D::CurvePlot *plotWidet = new Qwt3D::CurvePlot(this);
    plotWidet->createDataset();
    ui->scene3D = plotWidet;
    ui->showCenter->addWidget(ui->scene3D);

    createMenus();
    createToolBars();
    createActions();

    // systemtray icon
    createSystemTrayActions();
    createTrayIcon();
    trayIcon->show();

    // init ui
    showRandomSpline();
    ui->torence->setValue(easybezierInterpolator.GetTolerance());
    ui->controlPtSlider->setValue(displaySettings.controlPointSize);
    updateStatusBar();

    // other
    animationTimer = new QTimer();
    connect(animationTimer, SIGNAL(timeout()), SLOT(moveCurve()));

    fpsTimer = new QTimer();
    connect(fpsTimer, SIGNAL(timeout()), SLOT(updateFPS()));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete animationTimer;
    delete fpsTimer;
    delete scene;
    clearPoints();
}

void MainWindow::createMenus(void)
{
}

void MainWindow::createToolBars(void)
{

}

void MainWindow::show3DScene()
{
    ui->showCenter->setCurrentWidget(ui->scene3D);
}

void MainWindow::show2DScene()
{
    ui->showCenter->setCurrentWidget(ui->scene2D);
}

void MainWindow::about()
{
    static const char message[] =
            "<p><b>Qt Main Window Example</b></p>"

            "<p>This is a demonstration of the QMainWindow, QToolBar and "
            "QDockWidget classes.</p>"

            "<p>The tool bar and dock widgets can be dragged around and rearranged "
            "using the mouse or via the menu.</p>"

            "<p>Each dock widget contains a colored frame and a context "
            "(right-click) menu.</p>"

        #ifdef Q_OS_MAC
            "<p>On OS X, the \"Black\" dock widget has been created as a "
            "<em>Drawer</em>, which is a special kind of QDockWidget.</p>"
        #endif
            ;

    QMessageBox::about(this, tr("About MainWindows"), message);
}

/// fillKnotVector - fill \var knotVector with knots for uniform cubic B-spline
/// that passes through endpoints.
void MainWindow::fillKnotVector()
{
    int middleKnotNumber = pointsNumber - 4;
    knotVector.clear();
    for (int counter = 0; counter < 4; ++counter)
        knotVector.push_back(0.0);
    for (int counter = 1; counter <= middleKnotNumber; ++counter)
        knotVector.push_back(1.0 / (middleKnotNumber + 1) * counter);
    for (int counter = 0; counter < 4; ++counter)
        knotVector.push_back(1.0);
}

void MainWindow::showKnotVector()
{
    QString knots("knots: [");
    for (int i=0; i<knotVector.size()-1; ++i) {
        knots.append(QString("%1, ").arg(knotVector.at(i)));
    }
    knots.append(QString("%1]").arg(knotVector.last()));

    ui->knots->setText(knots);
}

/// interpolateCurve - calculate new control points with de Boor algorithm,
/// break curve into multiple Bezier curves and interpolate each Bezier curve.
void MainWindow::interpolateCurve()
{
    const int n = controlPoints.size() - 1;
    const int degree = 3;

    const double *knots = knotVector.constData();

    Point cps[100];
    for (int i = 0; i < controlPoints.size(); ++i) {
        cps[i].x = controlPoints.at(i)->x();
        cps[i].y = controlPoints.at(i)->y();
    }

//    bezierInterpolator.CalculateBoorNet(controlPoints, knotVector, boorNetPoints);
    int segnum = 0;
    Point bezierCPs[100 * (degree+1)];
    int ret = easybezierInterpolator.DecomposeCurve(n, degree, knots, cps, segnum, bezierCPs);
    if (ret != GeometryCompute::SUCCESS)  {
        qWarning("error!");
        return;
    }

    boorNetPoints.clear();
    for (int i = 0; i < segnum; ++i) {
        int num = degree + 1;
        for (int j = 0; j < num; ++j) {
            QPointF pt;
            pt.setX(bezierCPs[num*i + j].x);
            pt.setY(bezierCPs[num*i + j].y);
            boorNetPoints.append(pt);
        }
    }

    interpolatedPoints.clear();
    interpolatedPoints.push_back(*(controlPoints.first()));

    easyBezierInterpolatedPoints.clear();
    easyBezierInterpolatedPoints.append(*(controlPoints.first()));

    for (int counter = 0; counter < boorNetPoints.size() - 3; counter += 4) {
        bezierInterpolator.InterpolateBezier(boorNetPoints[counter],
                                             boorNetPoints[counter + 1],
                                             boorNetPoints[counter + 2],
                                             boorNetPoints[counter + 3],
                                             interpolatedPoints);


        int ret = 0;
        Point pts[4];
        for (int i = 0; i < 4; ++i) {
            pts[i].x = boorNetPoints[counter+i].x();
            pts[i].y = boorNetPoints[counter+i].y();
        }
        ControlPoints cpts;
        cpts.points = pts;
        cpts.degree = 3;

        PointArray ptout;
        ptout.validIndex = 0;
        ptout.totalNum = 1000;
        ptout.points = (Point *)malloc(ptout.totalNum * sizeof(Point));

        ret = easybezierInterpolator.InterpolateBezier(cpts, ptout);
        if (ret != GeometryCompute::SUCCESS)  {
            qWarning("Interpolate error %d.", ret);
        }

        QPointF tmp;
        for (int i = 0; i < ptout.validIndex; ++i) {
            tmp.setX(ptout.points[i].x);
            tmp.setY(ptout.points[i].y);
            easyBezierInterpolatedPoints.push_back(tmp);
        }

        free(ptout.points);

#if 0
        int size = sizeof(double)*1000;
        double *out = (double *)malloc(size);
        int ret = 0;
        int outsize = 0;
        double contrlPt[4*2] = {0.0};
        for (int i = 0; i < 4; ++i) {
            contrlPt[i*2] = boorNetPoints[counter+i].x();
            contrlPt[i*2+1] = boorNetPoints[counter+i].y();
        }

        int degree = 3;
        int dim = 2;
        ret = easybezierInterpolator.InterpolateBezier(contrlPt, degree, dim, out, size, &outsize);
        if (ret != GeometryCompute::SUCCESS)  {
            qWarning("Interpolate error %d.", ret);
        }

        QPointF tmp;
        for (int i = 0; i < outsize/(2*sizeof(double)); ++i) {
            tmp.setX(out[i*2]);
            tmp.setY(out[i*2 + 1]);
            easyBezierInterpolatedPoints.push_back(tmp);
        }

        free(out);
#endif
    }

    easyBezierInterpolatedPoints.push_back(*(controlPoints.last()));

    interpolatedPoints.push_back(*(controlPoints.last()));
}

/// clearPoints - delete all control points properly.
void MainWindow::clearPoints()
{
    for (int counter = 0; counter < controlPoints.size(); ++counter)
        delete controlPoints[counter];
    controlPoints.clear();
}

/// addControlPoint - adds control point within borders of \var graphicsView.
void MainWindow::addControlPoint()
{
    int x_border = ui->graphicsView->width();;
    int y_border = ui->graphicsView->height();
    int x = qrand() % x_border;
    int y = qrand() % y_border;
    controlPoints.push_back(new QPointF(x, y));
}

void MainWindow::showControlPoints()
{
    QString cps("controlpoints: ");

    for (QVector<QPointF*>::iterator pointIt = controlPoints.begin();
         pointIt != controlPoints.end(); ++pointIt) {
        cps.append(QString("(%01,%02)").arg((*pointIt)->x()).arg((*pointIt)->y()));
    }

    ui->controlpoints->setText(cps);
}

void MainWindow::updateStatusBar()
{
    QString content(QString("Fps %1, Control Points %2, Interpolated Points %3")
                    .arg(framesNumber).arg(pointsNumber).arg(interpolatedPoints.size()));

    ui->statusBar->showMessage(content);
}

/// showRandomSpline - generate random control points and show them.
void MainWindow::showRandomSpline()
{
    clearPoints();

    for (int counter = 0; counter < pointsNumber; ++counter) {
        addControlPoint();
    }

    clearSceneAndUpdateView();
}

void MainWindow::on_startStopButton_clicked()
{
    showRandomSpline();
}

/// updateView - calculate content of the scene based on control points and show
/// it in graphicsView.
void MainWindow::updateView(QPointF *skipPoint)
{
    QTime t;
    t.start();
    interpolateCurve();

    // Show easy interpolated curve.
    for (QPolygonF::iterator pointIt = easyBezierInterpolatedPoints.begin(),
         pointEnd = easyBezierInterpolatedPoints.end(); pointIt != pointEnd; ++pointIt) {

        if (displaySettings.showEasyBezierLine &&
            pointIt != easyBezierInterpolatedPoints.end() - 1) {
            scene->addLine(QLineF(*pointIt, *(pointIt + 1)), QPen("red"));
        }

        if (displaySettings.showEasyBezierInterpolatedPoints) {
            scene->addEllipse(pointIt->x() - 2, pointIt->y() - 2, 4, 4, QPen("black"),
                              QBrush("red"));
        }
    }

    // Show interpolated curve.
    for (QPolygonF::iterator pointIt = interpolatedPoints.begin(),
         pointEnd = interpolatedPoints.end(); pointIt != pointEnd; ++pointIt) {

        if (displaySettings.showBezierLine &&
            pointIt != interpolatedPoints.end() - 1) {
                scene->addLine(QLineF(*pointIt, *(pointIt + 1)), QPen("black"));
        }

        if (displaySettings.showInterpolatedPoints) {
            scene->addEllipse(pointIt->x() - 2, pointIt->y() - 2, 4, 4, QPen("black"),
                              QBrush("black"));
        }
    }

    // Show control points.
    for (QVector<QPointF*>::iterator pointIt = controlPoints.begin(),
         pointEnd = controlPoints.end(); pointIt != pointEnd; ++pointIt) {

        if (displaySettings.showControlLines && pointIt != controlPoints.end() - 1) {
            scene->addLine(QLineF(**pointIt, **(pointIt + 1)), QPen("blue"));
        }

        if ((skipPoint && skipPoint == *pointIt) ||
                !displaySettings.showControlPoints) {
            continue;
        }

        const int controlPointSize = displaySettings.controlPointSize;
        MovingEllipseItem *pointItem = new MovingEllipseItem(
                    (*pointIt)->x() - controlPointSize / 2,
                    (*pointIt)->y() - controlPointSize / 2,
                    controlPointSize, controlPointSize, this);
        itemToPoint[pointItem] = *pointIt;
        pointItem->setBrush(QBrush("blue"));
        pointItem->setFlag(QGraphicsItem::ItemIsMovable, true);
        pointItem->update();
        scene->addItem(pointItem);
    }
    // Show boor net points.
    for (QPolygonF::iterator pointIt = boorNetPoints.begin(),
         pointEnd = boorNetPoints.end(); pointIt != pointEnd; ++pointIt) {
        if (displaySettings.showBoorLines && pointIt != boorNetPoints.end() - 1)
            scene->addLine(QLineF(*pointIt, *(pointIt + 1)), QPen("red"));
        if (displaySettings.showBoorPoints)
            scene->addEllipse(pointIt->x() - 2, pointIt->y() - 2, 4, 4, QPen("green"),
                              QBrush("green"));
    }

    showKnotVector();
    showControlPoints();

    updateStatusBar();
    ++framesNumber;
}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    displaySettings.showInterpolatedPoints = arg1;
    clearSceneAndUpdateView();
}

void MainWindow::on_checkBox_2_stateChanged(int arg1)
{
    displaySettings.showControlPoints = arg1;
    clearSceneAndUpdateView();
}

void MainWindow::on_checkBox_3_stateChanged(int arg1)
{
    displaySettings.showBoorPoints = arg1;
    clearSceneAndUpdateView();
}

void MainWindow::on_checkBox_4_stateChanged(int arg1)
{
    displaySettings.showControlLines = arg1;
    clearSceneAndUpdateView();
}

void MainWindow::on_checkBox_5_stateChanged(int arg1)
{
    displaySettings.showBoorLines = arg1;
    clearSceneAndUpdateView();
}

void MainWindow::on_torence_valueChanged(double value)
{
    easybezierInterpolator.SetTolerance(value);
    clearSceneAndUpdateView();
}

void MainWindow::on_zoomInBtn_clicked()
{
    ui->graphicsView->scale(1.2, 1.2);
}

void MainWindow::on_zoomOutBtn_clicked()
{
    ui->graphicsView->scale(1/1.2, 1/1.2);
}

/// moveCurve - moves control points according to its speed and updates view.
void MainWindow::moveCurve()
{
    if (controlPointsSpeed.size() != controlPoints.size()) {
        // Randomly create speed.
        controlPointsSpeed.clear();
        const int speedLimit = 5;
        for (int counter = 0; counter < controlPoints.size(); ++counter) {
            controlPointsSpeed.push_back(QPointF(qrand() % speedLimit,
                                                 qrand() % speedLimit));
        }
    }

    // Move each control point.
    for (int counter = 0; counter < controlPoints.size(); ++counter) {
        QPointF &point = *controlPoints[counter];
        QPointF &speed = controlPointsSpeed[counter];
        double xSpeed = speedMultiplicator * speed.x();
        double ySpeed = speedMultiplicator * speed.y();
        point.setX(point.x() + xSpeed);
        point.setY(point.y() + ySpeed);
        if (point.x() < 0 || point.x() > ui->graphicsView->width()) {
            point.setX(point.x() - 2 * xSpeed);
            speed.setX(-speed.x());
        }
        if (point.y() < 0 || point.y() > ui->graphicsView->height()) {
            point.setY((point.y() - 2 * ySpeed));
            speed.setY(-speed.y());
        }
        if (point.x() < 0 || point.x() > ui->graphicsView->width() ||
                point.y() < 0 || point.y() > ui->graphicsView->height()) {
            // Looks like view was resized and point is out of view.
            point.setX(qrand() % ui->graphicsView->width());
            point.setY(qrand() % ui->graphicsView->height());
        }
    }

    clearSceneAndUpdateView();
}

void MainWindow::on_RandomButton_clicked()
{
    if (animationTimer->isActive()) {
        animationTimer->stop();
        fpsTimer->stop();
    } else {
        animationTimer->start(30);
        fpsTimer->start(1000);
    }
}

void MainWindow::on_AddPointButton_clicked()
{
    ++pointsNumber;
    addControlPoint();
    fillKnotVector();
    updateStatusBar();
    clearSceneAndUpdateView();
}

void MainWindow::on_DelPointButton_clicked()
{
    if (pointsNumber < 4)
        return;
    --pointsNumber;

    delete controlPoints.last();
    controlPoints.pop_back();
    showControlPoints();

    if (!controlPointsSpeed.empty())
        controlPointsSpeed.pop_back();
    fillKnotVector();
    updateStatusBar();
    clearSceneAndUpdateView();
}

/// updateFPS - show FPS in gui.
void MainWindow::updateFPS()
{
    updateStatusBar();
    framesNumber = 0;
}

void MainWindow::clearSceneAndUpdateView()
{
    scene->clear();
    updateView();
}

void MainWindow::on_AntialiasingSlider_sliderMoved(int position)
{
    Q_ASSERT(position >= 0 && position <= 2);
    switch (position) {
    case 0:
        // Disable antialiasing.
        ui->graphicsView->setRenderHint(QPainter::Antialiasing, false);
        ui->graphicsView->setRenderHint(QPainter::HighQualityAntialiasing, false);
        ui->AntialiasingLabel->setText("Antialiasing: None");
        break;
    case 1:
        // Enable antialiasing.
        ui->graphicsView->setRenderHint(QPainter::Antialiasing, true);
        ui->graphicsView->setRenderHint(QPainter::HighQualityAntialiasing, false);
        ui->AntialiasingLabel->setText("Antialiasing: Medium");
        break;
    case 2:
        // Maximum quality - maximum lags.
        ui->graphicsView->setRenderHint(QPainter::Antialiasing, true);
        ui->graphicsView->setRenderHint(QPainter::HighQualityAntialiasing, true);
        ui->AntialiasingLabel->setText("Antialiasing: High");
        break;
    }
}

void MainWindow::on_SpeedSlider_sliderMoved(int position)
{
    Q_ASSERT(position >= 1 && position <= 20);
    speedMultiplicator = (double) position / 10.0;
    ui->SpeedLabel->setText("Speed: " +
                            QString::number(speedMultiplicator, 'f', 1) + "x");
}

void MainWindow::on_AntialiasingSlider_valueChanged(int value)
{
    on_AntialiasingSlider_sliderMoved(value);
}

void MainWindow::on_SpeedSlider_valueChanged(int value)
{
    on_SpeedSlider_sliderMoved(value);
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    int max = ui->horizontalSlider->maximum();
    double distanceTolerance = (double) (max - position) + 0.5;
    bezierInterpolator.SetDistanceTolerance(distanceTolerance);

    QString prefix = "Interp Quality: ";
    QString postfix;
    // Divide quality into 4 ranges: Best, Good, Bad, Worst
    if (position >= 0 && position < max / 4)
        postfix = "Worst";
    else if (position >= max / 4 && position < max / 2)
        postfix = "Bad";
    else if (position >= max / 2 && position < 3 * max / 4)
        postfix = "Good";
    else
        postfix = "Best";

    ui->QualityLabel->setText(prefix + postfix);

     clearSceneAndUpdateView();
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    on_horizontalSlider_sliderMoved(value);
}

void MainWindow::on_controlPtSlider_sliderMoved(int position)
{
    on_controlPtSlider_valueChanged(position);
}

void MainWindow::on_controlPtSlider_valueChanged(int position)
{
    QString prefix = "Control Pointsize: ";
    ui->controlPtSizeLabel->setText(prefix + QString::number(position));
    displaySettings.controlPointSize = position;

    clearSceneAndUpdateView();
}

void MainWindow::createSystemTrayActions()
{
    restoreAction = new QAction(tr("&Restore"), this);
    connect(restoreAction, &QAction::triggered, this, &MainWindow::showNormal);

    quitAction = new QAction(tr("&Quit"), this);
    connect(quitAction, &QAction::triggered, qApp, &QCoreApplication::quit);
}

void MainWindow::createActions()
{
    connect(ui->actionExit, &QAction::triggered, qApp, &QApplication::quit);
    connect(ui->actionAbout, &QAction::triggered, this, &MainWindow::about);
    connect(ui->action2D, &QAction::triggered, this, &MainWindow::show2DScene);
    connect(ui->action3D, &QAction::triggered, this, &MainWindow::show3DScene);
}

void MainWindow::createTrayIcon()
{
    trayIconMenu = new QMenu(this);
    trayIconMenu->addAction(restoreAction);
    trayIconMenu->addSeparator();
    trayIconMenu->addAction(quitAction);

    trayIcon = new QSystemTrayIcon(this);
    trayIcon->setIcon(QIcon(":/icon/res/icon/typora.png"));
    trayIcon->setContextMenu(trayIconMenu);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
#ifdef Q_OS_OSX
    if (!event->spontaneous() || !isVisible()) {
        return;
    }
#endif
    if (trayIcon->isVisible()) {
        QMessageBox::StandardButton but =
                QMessageBox::question(this,
                                      tr("Quit or Hidden"),
                                      tr("To terminate the program, choose Yes, "
                                         "or The program will keep running in the system tray,"
                                         "and click the system tray to restore or quit." ));
        if (but == QMessageBox::Yes) {
            qApp->quit();
        } else {
            hide();
            event->ignore();
        }
    }
}
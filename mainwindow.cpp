#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "movingellipseitem.h"
#include <QTime>

#include <QSpinBox>
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
{
    // init data
    mNurbsCurve.n = 3;
    mNurbsCurve.p = 3;
    dspSettings.showBezierPoints = true;
    dspSettings.showBezierLine = true;
    dspSettings.showControlLines = true;
    dspSettings.showControlPoints = true;
    dspSettings.showBoorLines = false;
    dspSettings.showBoorPoints = false;
    dspSettings.easyBezierInterpolatedPnts = true;
    dspSettings.easyBezierLine = true;
    dspSettings.controlPointSize = 20;

    fillKnotVector();

    // creat ui.
    ui->setupUi(this);

    scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    delete ui->scene3D;
    m3DPlotWidet = new Qwt3D::CurvePlot(this);
    ui->scene3D = m3DPlotWidet;
    ui->showCenter->addWidget(ui->scene3D);

    createMenus();
    createToolBars();
    createActions();

    // systemtray icon
    createSystemTrayActions();
    createTrayIcon();
    trayIcon->show();

    // init ui
    ui->pointNum->setValue(mNurbsCurve.n + 1);
    ui->degree->setMaximum(mNurbsCurve.n);
    ui->degree->setValue(mNurbsCurve.p);

    ui->InterpPnt->setChecked(dspSettings.showBezierPoints);
    ui->InterpLine->setChecked(dspSettings.showBezierLine);
    ui->easyBezierPnt->setChecked(dspSettings.easyBezierInterpolatedPnts);
    ui->easyBezierLine->setChecked(dspSettings.easyBezierLine);
    ui->BoorPnt->setChecked(dspSettings.showBoorPoints);
    ui->BoorLine->setChecked(dspSettings.showBoorLines);
    ui->ControlPnt->setChecked(dspSettings.showControlPoints);
    ui->ControlLine->setChecked(dspSettings.showControlLines);

    ui->torence->setValue(easybezierInterpolator.GetTolerance());
    ui->torence->setMinimum(easybezierInterpolator.GetMinTolerance());
    ui->controlPtSlider->setValue(dspSettings.controlPointSize);

    // bind ui
    connect(ui->InterpPnt, &QCheckBox::stateChanged, this, &MainWindow::onInterpPnt_stateChanged);
    connect(ui->InterpLine, &QCheckBox::stateChanged, this, &MainWindow::onInterpLine_stateChanged);
    connect(ui->ControlPnt, &QCheckBox::stateChanged, this, &MainWindow::onControlPnt_stateChanged);
    connect(ui->ControlLine, &QCheckBox::stateChanged, this, &MainWindow::onControlLine_stateChanged);
    connect(ui->BoorPnt, &QCheckBox::stateChanged, this, &MainWindow::onBoorPnt_stateChanged);
    connect(ui->BoorLine, &QCheckBox::stateChanged, this, &MainWindow::onBoorLine_stateChanged);
    connect(ui->easyBezierPnt, &QCheckBox::stateChanged, this, &MainWindow::onEasyBezierPnt_stateChanged);
    connect(ui->easyBezierLine, &QCheckBox::stateChanged, this, &MainWindow::onEasyBezierLine_stateChanged);

    connect(ui->pointNum, SIGNAL(valueChanged(int)), this, SLOT(onPointNum_valueChanged(int)));
    connect(ui->degree, SIGNAL(valueChanged(int)), this, SLOT(onDegree_valueChanged(int)));
    connect(ui->torence, SIGNAL(valueChanged(double)), this, SLOT(onTorence_valueChanged(double)));
    connect(ui->controlPtSlider, SIGNAL(valueChanged(int)), this, SLOT(onControlPtSlider_valueChanged(int)));

    showRandomSpline();
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
            ;

    QMessageBox::about(this, tr("About MainWindows"), message);
}

/// fillKnotVector - fill \var knotVector with knots for uniform cubic B-spline
/// that passes through endpoints.
void MainWindow::fillKnotVector()
{
    int middleKnotNumber = mNurbsCurve.n - mNurbsCurve.p;
    knotVector.clear();

    for (int counter = 0; counter <= mNurbsCurve.p; ++counter)
        knotVector.push_back(0.0);

    for (int counter = 1; counter <= middleKnotNumber; ++counter)
        knotVector.push_back(1.0 / (middleKnotNumber + 1) * counter);

    for (int counter = 0; counter <= mNurbsCurve.p; ++counter)
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


void MainWindow::finalInterpolateCurve()
{
    mNurbsCurve.U = knotVector.data();
    mNurbsCurve.Pw =ctrlPnts.data();

    PointArray ptout;
    ptout.validIndex = 0;
    ptout.totalNum = 1000;
    ptout.points = (Point *)malloc(ptout.totalNum * sizeof(Point));

    int ret = easybezierInterpolator.DecomposeNurbsToLine(mNurbsCurve, ptout);
    if (ret != GeometryCompute::SUCCESS)  {
        qWarning("DecomposeNurbsToLine error: %d.", ret);
    }

    subdiviedPnts.clear();
    QPointF tmp;
    for (int i = 0; i < ptout.validIndex; ++i) {
        subdiviedPnts.push_back(ptout.points[i]);
    }

    free(ptout.points);
}

#if 0
void MainWindow::easyInterpolateCurve()
{
    const int n = ctrlPnts.size() - 1;
    const int degree = 3;

    const double *knots = knotVector.constData();

    Point cps[100];
    for (int i = 0; i < ctrlPnts.size(); ++i) {
        cps[i].x = ctrlPnts.at(i)->x();
        cps[i].y = ctrlPnts.at(i)->y();
    }

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

    subdiviedPnts.clear();
    subdiviedPnts.append(*(ctrlPnts.first()));

    for (int counter = 0; counter < boorNetPoints.size() - 3; counter += 4) {
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
            subdiviedPnts.push_back(tmp);
        }

        free(ptout.points);
    }

    subdiviedPnts.push_back(*(ctrlPnts.last()));
}
#endif

#if 0
/// interpolateCurve - calculate new control points with de Boor algorithm,
/// break curve into multiple Bezier curves and interpolate each Bezier curve.
void MainWindow::interpolateCurve()
{
    bezierInterpolator.CalculateBoorNet(ctrlPnts, knotVector, boorNetPoints);

    interpolatedPoints.clear();
    interpolatedPoints.push_back(*(ctrlPnts.first()));

    for (int counter = 0; counter < boorNetPoints.size() - 3; counter += 3) {
        bezierInterpolator.InterpolateBezier(boorNetPoints[counter],
                                             boorNetPoints[counter + 1],
                                             boorNetPoints[counter + 2],
                                             boorNetPoints[counter + 3],
                                             interpolatedPoints);
    }

    interpolatedPoints.push_back(*(ctrlPnts.last()));
}
#endif

/// clearPoints - delete all control points properly.
void MainWindow::clearPoints()
{
    ctrlPnts.clear();
}

/// addControlPoint - adds control point within borders of \var graphicsView.
void MainWindow::addControlPoint()
{
    int x_border = ui->graphicsView->width();;
    int y_border = ui->graphicsView->height();
    int z_border = ui->graphicsView->height();
    int x = qrand() % (x_border - 50);
    int y = qrand() % (y_border - 50);
    int z = qrand() % (z_border - 50);

    ctrlPnts.push_back(Point(x, y, z));
}

void MainWindow::showControlPoints()
{
    QString cps("controlpoints: ");

    int total = ctrlPnts.size();
    for (int i = 0; i < total; ++i) {
        cps.append(QString("(%01,%02, %03)")
                   .arg(ctrlPnts[i].x)
                   .arg(ctrlPnts[i].y)
                   .arg(ctrlPnts[i].z));
    }

    ui->controlpoints->setText(cps);
}

void MainWindow::updateStatusBar()
{
    QString content(QString("Fps %1, Control Points %2, Interpolated Points %3, %4")
                    .arg(framesNumber).arg(mNurbsCurve.n).arg(interpolatedPoints.size())
                    .arg(subdiviedPnts.size()));

    ui->statusBar->showMessage(content);
}

/// showRandomSpline - generate random control points and show them.
void MainWindow::showRandomSpline()
{
    clearPoints();

    for (int counter = 0; counter <= mNurbsCurve.n; ++counter) {
        addControlPoint();
    }

    clearAndUpdateAllView();
}

void MainWindow::on_startStopButton_clicked()
{
    showRandomSpline();
}

/// update2DView - calculate content of the scene based on control points and show
/// it in graphicsView.
void MainWindow::update2DView(int skipId)
{
    QTime t;
    t.start();

    // Show easy interpolated curve.

    int total = subdiviedPnts.size();
    for (int i = 0; i < total; ++i) {
        if (dspSettings.easyBezierLine && i != total -1) {
            scene->addLine(QLineF(QPointF(subdiviedPnts[i].x, subdiviedPnts[i].y),
                                  QPointF(subdiviedPnts[i+1].x, subdiviedPnts[i+1].y)),
                           QPen("red"));
        }

        if (dspSettings.easyBezierInterpolatedPnts) {
            scene->addEllipse(subdiviedPnts[i].x - 2, subdiviedPnts[i].y - 2, 4, 4, QPen("red"),
                              QBrush("red"));
        }
    }

    // Show interpolated curve.
    for (QPolygonF::iterator pointIt = interpolatedPoints.begin(),
         pointEnd = interpolatedPoints.end(); pointIt != pointEnd; ++pointIt) {

        if (dspSettings.showBezierLine &&
            pointIt != interpolatedPoints.end() - 1) {
                scene->addLine(QLineF(*pointIt, *(pointIt + 1)), QPen("black"));
        }

        if (dspSettings.showBezierPoints) {
            scene->addEllipse(pointIt->x() - 2, pointIt->y() - 2, 4, 4, QPen("black"),
                              QBrush("black"));
        }
    }

    // Show control points.
    total = ctrlPnts.size();
    for (int i = 0; i < total; ++i) {
        if (dspSettings.showControlLines && i != total - 1) {
            scene->addLine(QLineF(QPointF(ctrlPnts[i].x, ctrlPnts[i].y),
                                  QPointF(ctrlPnts[i+1].x, ctrlPnts[i+1].y)),
                           QPen("blue"));
        }

        if (dspSettings.showControlPoints && i != skipId) {
            const int controlPointSize = dspSettings.controlPointSize;
            QRectF size;
            size.setSize(QSize(controlPointSize, controlPointSize));
            size.moveCenter(QPoint(ctrlPnts[i].x, ctrlPnts[i].y));
            MovingEllipseItem *pointItem = new MovingEllipseItem(size, this);
            pointItem->setCtrlPntId(i);
            pointItem->setBrush(QBrush("blue"));
            pointItem->setFlag(QGraphicsItem::ItemIsMovable, true);
            pointItem->update();
            scene->addItem(pointItem);
        }
    }

    // Show boor net points.
    for (QPolygonF::iterator pointIt = boorNetPoints.begin(),
         pointEnd = boorNetPoints.end(); pointIt != pointEnd; ++pointIt) {
        if (dspSettings.showBoorLines && pointIt != boorNetPoints.end() - 1)
            scene->addLine(QLineF(*pointIt, *(pointIt + 1)), QPen("red"));
        if (dspSettings.showBoorPoints)
            scene->addEllipse(pointIt->x() - 2, pointIt->y() - 2, 4, 4, QPen("green"),
                              QBrush("green"));
    }

    showKnotVector();
    showControlPoints();

    updateStatusBar();
    ++framesNumber;
}

void MainWindow::onInterpPnt_stateChanged(int state)
{
    dspSettings.showBezierPoints = state;
    clearAndUpdateAllView();
}

void MainWindow::onInterpLine_stateChanged(int state)
{
    dspSettings.showBezierLine = state;
    clearAndUpdateAllView();
}

void MainWindow::onControlPnt_stateChanged(int state)
{
    dspSettings.showControlPoints = state;
    clearAndUpdateAllView();
}

void MainWindow::onBoorPnt_stateChanged(int state)
{
    dspSettings.showBoorPoints = state;
    clearAndUpdateAllView();
}

void MainWindow::onControlLine_stateChanged(int state)
{
    dspSettings.showControlLines = state;
    clearAndUpdateAllView();
}

void MainWindow::onBoorLine_stateChanged(int state)
{
    dspSettings.showBoorLines = state;
    clearAndUpdateAllView();
}

void MainWindow::onEasyBezierPnt_stateChanged(int state)
{
    dspSettings.easyBezierInterpolatedPnts = state;
    clearAndUpdateAllView();
}

void MainWindow::onEasyBezierLine_stateChanged(int state)
{
    dspSettings.easyBezierLine = state;
    clearAndUpdateAllView();
}

void MainWindow::onDegree_valueChanged(int value)
{
    if (value < 1) {
        return;
    }

    mNurbsCurve.p = value;
    fillKnotVector();
    clearAndUpdateAllView();
}

void MainWindow::onPointNum_valueChanged(int value)
{
    if (value <= 2) {
        return;
    }

    int diff = value - 1 - mNurbsCurve.n;
    mNurbsCurve.n = value - 1;
    if (diff == 0) {
        return;
    } else if (diff > 0) {
        for (int i = 0; i < diff; ++i) {
            addControlPoint();
        }
    } else if (diff < 0) {
        for (int i = 0; i < -diff; ++i) {
            ctrlPnts.pop_back();
        }
    }

    ui->degree->setMaximum(mNurbsCurve.n);

    fillKnotVector();
    updateStatusBar();
    clearAndUpdateAllView();
}

void MainWindow::onTorence_valueChanged(double value)
{
    easybezierInterpolator.SetTolerance(value);
    clearAndUpdateAllView();
}


/// moveCurve - moves control points according to its speed and updates view.
void MainWindow::moveCurve()
{
    if (controlPointsSpeed.size() != ctrlPnts.size()) {
        // Randomly create speed.
        controlPointsSpeed.clear();
        const int speedLimit = 5;
        for (int counter = 0; counter < ctrlPnts.size(); ++counter) {
            controlPointsSpeed.push_back(QPointF(qrand() % speedLimit,
                                                 qrand() % speedLimit));
        }
    }

    // Move each control point.
    int total = ctrlPnts.size();
    for (int counter = 0; counter < total; ++counter) {
        QPointF point(ctrlPnts[counter].x, ctrlPnts[counter].y);
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

    clearAndUpdateAllView();
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

//void MainWindow::on_AddPointButton_clicked()
//{
//    ++mNurbsCurve.n;
//    addControlPoint();
//    fillKnotVector();
//    updateStatusBar();
//    clearAndUpdateAllView();
//}

//void MainWindow::on_DelPointButton_clicked()
//{
//    if (mNurbsCurve.n < 3)
//        return;
//    --mNurbsCurve.n;

//    delete ctrlPnts.last();
//    ctrlPnts.pop_back();
//    showControlPoints();

//    if (!controlPointsSpeed.empty())
//        controlPointsSpeed.pop_back();
//    fillKnotVector();
//    updateStatusBar();
//    clearAndUpdateAllView();
//}

/// updateFPS - show FPS in gui.
void MainWindow::updateFPS()
{
    updateStatusBar();
    framesNumber = 0;
}

void MainWindow::clearAndUpdateAllView()
{
    finalInterpolateCurve();

    scene->clear();
    update2DView();

    update3DWidget();
}

void MainWindow::update3DWidget()
{
    Qwt3D::TripleVector data;
    for (int i = 0; i < subdiviedPnts.size(); ++i) {
        Point pt(subdiviedPnts.at(i));
        data.push_back(Qwt3D::Triple(pt.x, pt.y, pt.z));
    }
    m3DPlotWidet->setPointsOnCurve(data);

    double *pw = new double [4*(mNurbsCurve.n+1)];
    for (int i = 0; i < mNurbsCurve.n+1; ++i) {
        pw[4*i] = mNurbsCurve.Pw[i].x;
        pw[4*i+1] = mNurbsCurve.Pw[i].y;
        pw[4*i+2] = mNurbsCurve.Pw[i].z;
        pw[4*i+3] = mNurbsCurve.Pw[i].w;
    }
    m3DPlotWidet->setNurbsData(mNurbsCurve.n+1+mNurbsCurve.p+1,
                               mNurbsCurve.U, 4, pw, mNurbsCurve.p+1);
    delete [] pw;

    m3DPlotWidet->updateData();
    m3DPlotWidet->update();
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

     clearAndUpdateAllView();
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    on_horizontalSlider_sliderMoved(value);
}

void MainWindow::on_controlPtSlider_sliderMoved(int position)
{
    onControlPtSlider_valueChanged(position);
}

void MainWindow::onControlPtSlider_valueChanged(int position)
{
    QString prefix = "Control Pointsize: ";
    ui->controlPtSizeLabel->setText(prefix + QString::number(position));
    dspSettings.controlPointSize = position;

    clearAndUpdateAllView();
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

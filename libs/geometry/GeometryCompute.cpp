#include "GeometryCompute.h"

void PointArray::AddPoint(Point &Point)
{
    if (usedIndex < totalNum - 1)
    {
        points[usedIndex] = Point;
        usedIndex++;
    }
}

void Point::MidPoint(const Point &Pt1, const Point &Pt2)
{
    x = (Pt1.x + Pt2.x)/2;
    y = (Pt1.y + Pt2.y)/2;
    z = (Pt1.z + Pt2.z)/2;
}

double Point::DistanceToLine(const Point &Pt,
                          const Point &P1,
                          const Point &P2)
{
    Point P1P2;
    Point P1Pt;
    Point tmp;
    double P1P2Dist = 0.0;   // P1P2线段长度的平方
    double T = 0.0;  // P1Pt 点积 P1P2

    P1P2 = P2 - P1;
    P1Pt = Pt - P1;

    P1P2Dist = dotProduct(P1P2, P1P2);

    if (isNull(P1P2Dist))
    {
        return dotProduct(P1Pt, P1Pt);
    }

    T = dotProduct(P1P2, P1Pt);
    T /= P1P2Dist;  // 系数t

    tmp = P1 + T*P1P2 - Pt;
    return dotProduct(tmp, tmp);
}

GeometryCompute::GeometryCompute()
{

}

int GeometryCompute::InterpolateBezier(ControlPoints ControlPts, PointArray &PtOut)
{
    int ret = SUCCESS;

    PtOut.usedIndex = 0;
    PtOut.AddPoint(ControlPts.points[0]);

    if (ControlPts.degree == 2)
    {
        ret = Recursive2DegreeBezier(ControlPts.points[0], ControlPts.points[2], ControlPts.points[4], 0, PtOut);
    }
    else if (ControlPts.degree == 3)
    {
        ret = Recursive3DegreeBezier(ControlPts.points[0], ControlPts.points[1], ControlPts.points[2], ControlPts.points[3],
                0, PtOut);
    } else {
        return DEGREE_INVALID;
    }

    if (ret == SUCCESS)
    {
        PtOut.AddPoint(ControlPts.points[ControlPts.degree]);
    }

    return ret;
}

/**
 * @brief GeometryCompute::DistanceToLine
 * @param Pt 输入: 直线外的点
 * @param LineP1 输入: 直线上的点P1
 * @param LineP2 输入: 直线上的点P2
 * @param Dim 输入: 点的维数
 * @param Distance 输出: 点Pt到直线P1P2的距离的平方
 * @return 返回操作结果状态码 ERRCODE
 *  算法：假设Pt与直线P1P2的垂线的交点为Pc，先求系数 t=|PcP1|/|P1P2|, 然后求Pc坐标，
 *     最后求|PCP1|
 */
int GeometryCompute::DistanceToLine(const double Pt[], const double P1[], const double P2[], int Dim,
                                    double *Distance)
{
    double P1P2[MAX_DIMENSION] = {0.0};
    double P1Pt[MAX_DIMENSION] = {0.0};
    double P1P2Dist = 0.0;   // P1P2线段长度的平方
    double T = 0.0;  // P1Pt 点积 P1P2
    double tmp = 0.0;

    if (Dim > MAX_DIMENSION)
    {
        return DIM_INVALID;
    }

    for (int i = 0; i < Dim; ++i)
    {
        P1P2[i] = P2[i] - P1[i];
        P1Pt[i] = Pt[i] - P1[i];
    }

    for (int i = 0; i < Dim; ++i)
    {
        P1P2Dist += P1P2[i]*P1P2[i];
        T += P1P2[i]*P1Pt[i];
    }

    if (P1P2Dist < 0.001)
    {
        return PARA_INVALID;
    }

    T /= P1P2Dist;  // 系数t

    for (int i = 0; i < Dim; ++i)
    {
        tmp = P1[i] + T*P1P2[i] - Pt[i]; // PtPc坐标分量差, 其中Pc坐标为LineP1[i] + T*P1P2[i]
        *Distance += tmp*tmp;
    }

    return SUCCESS;
}

int GeometryCompute::InterpolateBezier(double BezierControlPts[], int Degree, int Dim,
                                       double PtOut[], int Size, int *UsedSize)
{
    int ret = SUCCESS;
    mPtMem = PtOut;
    mPtTotalSize = Size;
    mPtIndex = 0;
    if (Degree < 2 || Degree > 3) {
        return DEGREE_INVALID;
    }

    ret = AddPoint(&BezierControlPts[0], Dim);
    if (ret != SUCCESS)
    {
        return NOT_ENOUGH_MEMERY;
    }

    if (Degree == 2)
    {
        ret = Recursive2DegreeBezier(&BezierControlPts[0], &BezierControlPts[2], &BezierControlPts[4], 0);
    }
    else
    {
        ret = Recursive3DegreeBezier(&BezierControlPts[0*Dim], &BezierControlPts[1*Dim], &BezierControlPts[2*Dim], &BezierControlPts[3*Dim], Dim, 0);
    }

    if (ret != SUCCESS)
    {
        return ret;
    }

    ret = AddPoint(&BezierControlPts[Degree*Dim], Dim);
    if (ret != SUCCESS)
    {
        return NOT_ENOUGH_MEMERY;
    }

    *UsedSize = mPtIndex * sizeof(double);

    return SUCCESS;
}

int GeometryCompute::Recursive2DegreeBezier(double Pt1[3], double Pt2[3], double Pt3[3], int Level)
{
    return SUCCESS;
}

void GeometryCompute::MidPoint(double P1[], double P2[], double PMid[], int Dim)
{
    for (int i = 0; i < Dim; ++i)
    {
        PMid[i] = (P1[i] + P2[i]) / 2;
    }
}

int GeometryCompute::Recursive3DegreeBezier(double Pt1[],
                                            double Pt2[],
                                            double Pt3[],
                                            double Pt4[],
                                            int Dim,
                                            int Level)
{
    int ret = SUCCESS;
    double Pt12[MAX_DIMENSION] = {0.0};
    double Pt23[MAX_DIMENSION] = {0.0};
    double Pt34[MAX_DIMENSION] = {0.0};
    double Pt123[MAX_DIMENSION] = {0.0};
    double Pt234[MAX_DIMENSION] = {0.0};
    double Pt1234[MAX_DIMENSION] = {0.0};

    double Dis12 = {0.0};
    double Dis23 = {0.0};

    if(Level > CURVE_RECURSION_LIMIT)
    {
        return RECURSIVE_OVERFLOW;
    }

    // Calculate all the mid-points of the line segments
    MidPoint(Pt1, Pt2, Pt12, Dim);
    MidPoint(Pt2, Pt3, Pt23, Dim);
    MidPoint(Pt3, Pt4, Pt34, Dim);
    MidPoint(Pt12, Pt23, Pt123, Dim);
    MidPoint(Pt23, Pt34, Pt234, Dim);
    MidPoint(Pt123, Pt234, Pt1234, Dim);

    if (Level > 0) {
        // Try to approximate the full cubic curve by a single straight line
        DistanceToLine(Pt2, Pt1, Pt4, Dim, &Dis12); // ToDo
        DistanceToLine(Pt3, Pt1, Pt4, Dim, &Dis23);

        if(Dis12 + Dis23 < mDistanceTolerance)
        {
            return AddPoint(Pt1234, Dim);
        }
    }

    // Continue subdivision
    ret = Recursive3DegreeBezier(Pt1, Pt12, Pt123, Pt1234, Dim, Level+1);
    if (ret != SUCCESS)
    {
        return ret;
    }

    ret = Recursive3DegreeBezier(Pt1234, Pt234, Pt34, Pt4, Dim, Level+1);
    if (ret != SUCCESS)
    {
        return ret;
    }

    return SUCCESS;
}

int GeometryCompute::Recursive2DegreeBezier(Point Pt1, Point Pt2, Point Pt3, int Level, PointArray &PtOut)
{
    return SUCCESS;
}

int GeometryCompute::Recursive3DegreeBezier(Point Pt1, Point Pt2, Point Pt3, Point Pt4, int Level, PointArray &PtOut)
{
    int ret = SUCCESS;
    Point Pt12, Pt23, Pt34;
    Point Pt123, Pt234;
    Point Pt1234;

    double Dis12 = {0.0};
    double Dis23 = {0.0};

    if(Level > CURVE_RECURSION_LIMIT)
    {
        return RECURSIVE_OVERFLOW;
    }

    // Calculate all the mid-points of the line segments
    Pt12.MidPoint(Pt1, Pt2);
    Pt23.MidPoint(Pt2, Pt3);
    Pt34.MidPoint(Pt3, Pt4);

    Pt123.MidPoint(Pt12, Pt23);
    Pt234.MidPoint(Pt23, Pt34);

    Pt1234.MidPoint(Pt123, Pt234);

    if (Level > 0) {
        // Try to approximate the full cubic curve by a single straight line
        Dis12 = Point::DistanceToLine(Pt2, Pt1, Pt4);
        Dis23 = Point::DistanceToLine(Pt3, Pt1, Pt4);

        if(Dis12 + Dis23 < mDistanceTolerance)
        {
            PtOut.AddPoint(Pt1234);
            return SUCCESS;
        }
    }

    // Continue subdivision
    ret = Recursive3DegreeBezier(Pt1, Pt12, Pt123, Pt1234, Level+1, PtOut);
    if (ret != SUCCESS)
    {
        return ret;
    }

    ret = Recursive3DegreeBezier(Pt1234, Pt234, Pt34, Pt4, Level+1, PtOut);
    if (ret != SUCCESS)
    {
        return ret;
    }

    return SUCCESS;
}

int GeometryCompute::AddPoint(double Pt[], int Dim)
{
    if (mPtIndex+Dim-1 >= mPtTotalSize/sizeof(double))
    {
        return NOT_ENOUGH_MEMERY;
    }

    for (int i = 0; i < Dim; ++i)
    {
        mPtMem[mPtIndex+i] = Pt[i];
    }

    mPtIndex += Dim;

    return SUCCESS;
}

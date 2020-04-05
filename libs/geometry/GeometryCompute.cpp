#include "GeometryCompute.h"
#include <float.h>

int PointArray::AddPoint(Point &Point)
{
    if (validIndex < totalNum - 1)
    {
        points[validIndex] = Point;
        validIndex++;
        return 0;
    }
    else
    {
        return -1;
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

void GeometryCompute::SetTolerance(double tolerance)
{
    mDistanceTolerance = mDistanceTolerance < FLT_MIN ? FLT_MIN : tolerance;
}

int GeometryCompute::InterpolateBezier(ControlPoints ControlPts, PointArray &PtOut)
{
    int Ret = SUCCESS;

    PtOut.validIndex = 0;
    PtOut.AddPoint(ControlPts.points[0]);

    if (ControlPts.degree == 2)
    {
        Ret = Recursive2DegreeBezier(ControlPts.points[0], ControlPts.points[2], ControlPts.points[4], 0, PtOut);
    }
    else if (ControlPts.degree == 3)
    {
        Ret = Recursive3DegreeBezier(ControlPts.points[0], ControlPts.points[1], ControlPts.points[2], ControlPts.points[3],
                0, PtOut);
    } else {
        return DEGREE_INVALID;
    }

    if (Ret == SUCCESS)
    {
        PtOut.AddPoint(ControlPts.points[ControlPts.degree]);
    }

    return Ret;
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
    int Ret = SUCCESS;
    mPtMem = PtOut;
    mPtTotalSize = Size;
    mPtIndex = 0;
    if (Degree < 2 || Degree > 3) {
        return DEGREE_INVALID;
    }

    Ret = AddPoint(&BezierControlPts[0], Dim);
    if (Ret != SUCCESS)
    {
        return NOT_ENOUGH_MEMERY;
    }

    if (Degree == 2)
    {
        Ret = Recursive2DegreeBezier(&BezierControlPts[0], &BezierControlPts[2], &BezierControlPts[4], 0);
    }
    else
    {
        Ret = Recursive3DegreeBezier(&BezierControlPts[0*Dim], &BezierControlPts[1*Dim], &BezierControlPts[2*Dim], &BezierControlPts[3*Dim], Dim, 0);
    }

    if (Ret != SUCCESS)
    {
        return Ret;
    }

    Ret = AddPoint(&BezierControlPts[Degree*Dim], Dim);
    if (Ret != SUCCESS)
    {
        return NOT_ENOUGH_MEMERY;
    }

    *UsedSize = mPtIndex * sizeof(double);

    return SUCCESS;
}

int GeometryCompute::DecomposeNurbsToLine(const Nurbs &Nurbs, PointArray &PtOut)
{
    int Ret = SUCCESS;
    int n = Nurbs.n;
    int p = Nurbs.p;
    Point *Pw = Nurbs.Pw;
    double *U = Nurbs.U;

    const int m = n + p + 1;    /* 节点数 */
    int a = p;         /* 节点Ua在重复组中(重复度p)最右端出现时的下标. */
    int b = p + 1;     /* 节点Ub在紧随Ua之后，重复组中最右端的下标(重复度还不到p). */

    if (p > MAX_DEGREE)
    {
        return DEGREE_TOO_HIGH;
    }

    PtOut.validIndex = 0;
    Point Qw[MAX_DEGREE];
    Point QwNext[MAX_DEGREE];

    for (int i = 0; i <= p; ++i)
    {
        Qw[i] = Pw[i];
    }

    while (b < m)
    {
        int i = b;
        while (b < m && U[b+1] == U[b])
        {
            b++;
        }
        int mult = b - i + 1;           /* 找出重复度, 如果只有一个则重复度为 1 */

        if (mult < p)
        {
            double number = U[b] - U[a];   /* 分子都相等 */

            /* 计算并存储 alphas */
            double alphas[100] = {0.0};
            for (int j = p; j > mult; --j)
            {
                alphas[j - mult -1] = number / (U[a + j] - U[a]); /* 东南对角线上，各alpha相等 */
            }

            int r = p - mult;               /* 插入节点的次数 */
            for (int j = 1; j <= r; ++j)    /* 执行 r 次插入 */
            {
                int s = mult + j;           /* s个新控制点 */
                for (int k = p; k >= s; --k)
                {
                    double alpha = alphas[k - s];
                    Qw[k] = alpha * Qw[k] + (1 - alpha) * Qw[k - 1];
                }

                if (b < m)                 /* 下一段的控制点 */
                {
                    int save = r - j;
                    QwNext[save] = Qw[p];
                }
            }
        }

        /* 重复度为p，说明当前bezier段处理完毕, 对bezier段其进行细分 */
        Ret = InterpolateBezier(p, Qw, PtOut);
        if (Ret != SUCCESS)
        {
            return Ret;
        }

        if (b < m)            /* 为下一段进行初始化 */
        {
            for (int i = 0; i <= p; ++i)
            {
                if (i < p - mult)
                {
                    Qw[i] = QwNext[i];
                }
                else
                {
                    Qw[i] = Pw[b - p +i];
                }
            }

            a = b;
            b++;
        }
    }


    return  SUCCESS;
}

int GeometryCompute::InterpolateBezier(int Degree, Point *ControlPts, PointArray &PtOut)
{
    int Ret = SUCCESS;

    Ret = PtOut.AddPoint(ControlPts[0]);
    if (Ret != 0)
    {
        return NOT_ENOUGH_MEMERY;
    }

    Ret = RecursiveBezier(ControlPts, Degree, 0, PtOut);
    if (Ret != SUCCESS)
    {
        return Ret;
    }

    Ret = PtOut.AddPoint(ControlPts[Degree]);
    if (Ret != 0)
    {
        return NOT_ENOUGH_MEMERY;
    }

    return Ret;
}

int GeometryCompute::RecursiveBezier(Point Pts[], int Degree, int Level, PointArray &PtOut)
{
    int Ret = SUCCESS;
    double DisMax = 0.0;
    double Tmp = 0.0;
    Point PtMid1[MAX_DEGREE];
    Point PtMid2[MAX_DEGREE];

    if(Level > CURVE_RECURSION_LIMIT)
    {
        return RECURSIVE_OVERFLOW;
    }

    /* 计算最大偏差距离的平方 */
    for (int i = 1; i < Degree; ++i)
    {
        Tmp = Point::DistanceToLine(Pts[i], Pts[0], Pts[Degree]);
        if (Tmp > DisMax)
        {
            DisMax = Tmp;
        }
    }

    /* 计算所有的中点 */
    PtMid1[0] = Pts[0];
    PtMid2[Degree] = Pts[Degree];
    for (int i = 1; i <= Degree; ++i)
    {
        for (int j = 0; j <= Degree - i; ++j)
        {
            Pts[j] = (Pts[j] + Pts[j+1])/2;
        }
        PtMid1[i] = Pts[0];
        PtMid2[Degree - i] = Pts[Degree - i];
    }

    if (Level > 0) {
        if(DisMax < mDistanceTolerance)
        {
            Ret = PtOut.AddPoint(Pts[0]);
            if (Ret != 0)
            {
                return NOT_ENOUGH_MEMERY;
            }

            return SUCCESS;
        }
    }

    /* 继续细分 */
    Ret = RecursiveBezier(PtMid1, Degree, Level+1, PtOut);
    if (Ret != SUCCESS)
    {
        return Ret;
    }

    Ret = RecursiveBezier(PtMid2, Degree, Level+1, PtOut);
    if (Ret != SUCCESS)
    {
        return Ret;
    }

    return SUCCESS;
}


/*!
 * \brief DecomposeCurve
 * 功能: 将Nurbs曲线分为bezier曲线段.
 * 算法源自 The Nurbs Book A5.6.
 * \param n 控制点数-1
 * \param p 次数
 * \param U 节点数组 [U0, U1, ... ... Un]
 * \param Pw 控制点数组
 * \param nb 输出，bezier段数
 * \param Qw 输出, Qw[j][k]返回第j段第k个控制点
 * \return
 */
int GeometryCompute::DecomposeCurve(int n, int p, const double *U, const Point *Pw,
                                    int &nb, Point *Qw)
{
    const int m = n + p + 1;    /* 节点数 */
    const int num = p + 1;      /*       */
    int a = p;         /* 节点Ua在重复组中(重复度p)最右端出现时的下标. */
    int b = p + 1;     /* 节点Ub在紧随Ua之后，重复组中最右端的下标(重复度还不到p). */

    nb = 0;

    for (int i = 0; i <= p; ++i) {
        Qw[nb*num + i] = Pw[i];
    }

    while (b < m) {
        int i = b;
        while (b < m && U[b+1] == U[b]) {
            b++;
        }
        int mult = b - i + 1;           /* 找出重复度, 如果只有一个则重复度为 1 */

        if (mult < p) {
            double number = U[b] - U[a];   /* 分子都相等 */

            /* 计算并存储 alphas */
            double alphas[100] = {0.0};
            for (int j = p; j > mult; --j) {
                alphas[j - mult -1] = number / (U[a + j] - U[a]); /* 东南对角线上，各alpha相等 */
            }

            int r = p - mult;               /* 插入节点的次数 */
            for (int j = 1; j <= r; ++j) {  /* 执行 r 次插入 */
                int s = mult + j;           /* s个新控制点 */
                for (int k = p; k >= s; --k) {
                    double alpha = alphas[k - s];
                    Qw[nb*num + k] = alpha * Qw[nb*num + k] + (1 - alpha) * Qw[nb*num + k - 1];
                }

                if (b < m) {                /* 下一段的控制点 */
                    int save = r - j;
                    Qw[(nb+1)*num + save] = Qw[nb*num + p];
                }
            }
        }

        /* 重复度为p，说明当前bezier段处理完毕 */
        nb = nb + 1;         /* 更新bezier段数 */
        if (b < m) {           /* 为下一段进行初始化 */
            for (int i = p - mult; i <= p; ++i) {
                Qw[nb*num + i] = Pw[b - p + i];
            }

            a = b;
            b++;
        }
    }

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
    int Ret = SUCCESS;
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
    Ret = Recursive3DegreeBezier(Pt1, Pt12, Pt123, Pt1234, Dim, Level+1);
    if (Ret != SUCCESS)
    {
        return Ret;
    }

    Ret = Recursive3DegreeBezier(Pt1234, Pt234, Pt34, Pt4, Dim, Level+1);
    if (Ret != SUCCESS)
    {
        return Ret;
    }

    return SUCCESS;
}

int GeometryCompute::Recursive2DegreeBezier(Point Pt1, Point Pt2, Point Pt3, int Level, PointArray &PtOut)
{
    return SUCCESS;
}

int GeometryCompute::Recursive3DegreeBezier(Point Pt1, Point Pt2, Point Pt3, Point Pt4, int Level, PointArray &PtOut)
{
    int Ret = SUCCESS;
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
    Ret = Recursive3DegreeBezier(Pt1, Pt12, Pt123, Pt1234, Level+1, PtOut);
    if (Ret != SUCCESS)
    {
        return Ret;
    }

    Ret = Recursive3DegreeBezier(Pt1234, Pt234, Pt34, Pt4, Level+1, PtOut);
    if (Ret != SUCCESS)
    {
        return Ret;
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

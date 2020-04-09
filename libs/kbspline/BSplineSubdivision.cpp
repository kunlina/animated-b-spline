#include "BSplineSubdivision.hpp"

#include <stdio.h>

using namespace KBSpline;

int PointArray::AddPoint(Point &Point)
{
    if (ValidIndex < TotalNum - 1)
    {
        Points[ValidIndex] = Point;
        ValidIndex++;
        return 0;
    }
    else
    {
        return -1;
    }
}

/**
 * @brief BSplineSubdivision::DistanceToLine
 * @param Pt 输入: 直线外的点
 * @param LineP1 输入: 直线上的点P1
 * @param LineP2 输入: 直线上的点P2
 * @param Dim 输入: 点的维数
 * @param Distance 输出: 点Pt到直线P1P2的距离的平方
 * @return 返回操作结果状态码 ERRCODE
 *  算法：假设Pt与直线P1P2的垂线的交点为Pc，先求系数 t=|PcP1|/|P1P2|, 然后求Pc坐标，
 *     最后求|PCP1|
 */
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

    P1P2Dist = DotProduct(P1P2, P1P2);

    if (IsNull(P1P2Dist))
    {
        return DotProduct(P1Pt, P1Pt);
    }

    T = DotProduct(P1P2, P1Pt);
    T /= P1P2Dist;  // 系数t

    tmp = P1 + T*P1P2 - Pt;
    return DotProduct(tmp, tmp);
}

BSplineSubdivision::BSplineSubdivision(int RecursionLimit, double DefaultTolerance, double MiniTolerance)
    : CURVE_RECURSION_LIMIT(RecursionLimit)
    , MIN_TOLERANCE(MiniTolerance)
    , mDistanceTolerance(DefaultTolerance)
{
}

void BSplineSubdivision::SetTolerance(double tolerance)
{
    mDistanceTolerance = mDistanceTolerance < MIN_TOLERANCE*MIN_TOLERANCE ? MIN_TOLERANCE*MIN_TOLERANCE: tolerance;
}

int BSplineSubdivision::DecomposeNurbsToLine(const BSpline &BSpline, PointArray &PtOut)
{
    int Ret = SUCCESS;
    int n = BSpline.n;
    int p = BSpline.p;
    Point *Pw = BSpline.Pw;
    double *U = BSpline.U;

    const int m = n + p + 1;    /* 节点数 */
    int a = p;         /* 节点Ua在重复组中(重复度p)最右端出现时的下标. */
    int b = p + 1;     /* 节点Ub在紧随Ua之后，重复组中最右端的下标(重复度还不到p). */

    Ret = CheckKnotvector(BSpline);
    if (Ret != SUCCESS)
    {
        return Ret;
    }

    PtOut.ValidIndex = 0;
    Point Qw[MAX_DEGREE+1];
    Point QwNext[MAX_DEGREE+1];

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

int BSplineSubdivision::InterpolateBezier(int Degree, Point *ControlPts, PointArray &PtOut)
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

int BSplineSubdivision::RecursiveBezier(Point Pts[], int Degree, int Level, PointArray &PtOut)
{
    int Ret = SUCCESS;
    double DisMax = 0.0;
    double Tmp = 0.0;
    Point PtMid1[MAX_DEGREE+1];
    Point PtMid2[MAX_DEGREE+1];

    if(Level > CURVE_RECURSION_LIMIT)
    {
        printf("curve %d\n", Level);
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
            Point pt = (Pts[j] + Pts[j+1]);
            Pts[j] = pt/2;
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

int BSplineSubdivision::CheckKnotvector(const BSpline &BSpline)
{
    int Kindex = BSpline.n;
    int Degree = BSpline.p;
    int ControlPntNum = BSpline.n + 1;

    int Order = Degree + 1;
    int KnotNum = ControlPntNum + Order;
    int Mult = 1;

    if (BSpline.p < MIN_DEGREE || BSpline.p > MAX_DEGREE)
    {
        return DEGREE_INVALID;
    }

    if (ControlPntNum < Order) {
        return N_LESS_THAN_P;
    }

    for (int i = 0; i < KnotNum - 1; ++i)
    {
        if (BSpline.U[i] > BSpline.U[i+1])
        {
            return KNOTS_DECREASING;
        }
        else if (BSpline.U[i] == BSpline.U[i+1])
        {
            Mult++;
            if (Mult > Order)
            {
                return MULTI_GREATER_THAN_ORDER;
            }
        }
        else
        {
            Mult = 1;
        }
    }

    for (int i = 0; i < Degree - 1; ++i)
    {
        if (BSpline.U[i] != BSpline.U[i + 1])
        {
            return KNOTS_IS_UNCLAMPED;
        }

        if (BSpline.U[KnotNum - 1 - i] != BSpline.U[KnotNum - 1 - i - 1])
        {
            return KNOTS_IS_UNCLAMPED;
        }
    }

    return SUCCESS;
}


/*!
 * \brief DecomposeCurve
 * 功能: 将Nurbs曲线分为bezier曲线段.
 * 算法源自 The BSpline Book A5.6.
 * \param n 控制点数-1
 * \param p 次数
 * \param U 节点数组 [U0, U1, ... ... Un]
 * \param Pw 控制点数组
 * \param nb 输出，bezier段数
 * \param Qw 输出, Qw[j][k]返回第j段第k个控制点
 * \return
 */
int BSplineSubdivision::DecomposeCurve(int n, int p, const double *U, const Point *Pw,
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

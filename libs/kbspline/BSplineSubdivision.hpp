#pragma once

namespace KBSpline {

struct Point
{
    Point(): x(0.0), y(0.0), z(0.0), w(0.0) {}
    Point(double x, double y, double z, double w = 1.0): x(x), y(y), z(z), w(w) {}
    double x, y, z, w;
    inline Point &operator+=(const Point &p)
    {
        x+=p.x; y+=p.y; z+=p.z; w+=p.w; return *this;
    }

    inline Point &operator-=(const Point &p)
    {
        x-=p.x; y-=p.y; z-=p.z; w-=p.w; return *this;
    }

    inline Point &operator*=(double c)
    {
        x*=c; y*=c; z*=c; w*=c; return *this;
    }

    inline Point &operator/=(double c)
    {
        x/=c; y/=c; z/=c; w/=c; return *this;
    }

    friend inline const Point operator+(const Point &p)
    {
        return p;
    }

    friend inline const Point operator-(const Point &p)
    {
        return Point(-p.x, -p.y, -p.z, -p.w);
    }

    friend inline const Point operator+(const Point &p1, const Point &p2)
    {
        return Point(p1.x+p2.x, p1.y+p2.y, p1.z+p2.z, p1.w+p2.w);
    }

    friend inline const Point operator-(const Point &p1, const Point &p2)
    {
        return Point(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z, p1.w-p2.w);
    }

    friend inline const Point operator*(const Point &p, double c)
    {
        return Point(p.x*c, p.y*c, p.z*c, p.w*c);
    }

    friend inline const Point operator*(double c, const Point &p)
    {
        return Point(p.x*c, p.y*c, p.z*c, p.w*c);
    }

    friend inline const Point operator/(const Point &p, double divisor)
    {
        return Point(p.x/divisor, p.y/divisor, p.z/divisor, p.w/divisor);
    }

    static inline double DotProduct(const Point &p1, const Point &p2)
    { return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z + p1.w*p2.w; }


    static inline bool IsNull(double d)
    {
        if (d <= 0.000000000001 && d >= -0.000000000001) {
            return true;
        }
        return false;
    }

    static inline bool IsNull(float d)
    {
        if (d <=  0.00001f && d >= -0.00001f) {
            return true;
        }
        return false;
    }

    static double DistanceToLine(const Point &Pt, const Point &P1, const Point &P2);
};

/* 输出的小线段 */
struct PointArray {
    int ValidIndex;         // 已经使用的个数
    int TotalNum;           // points空间的总长度
    Point *Points;

public:
    int AddPoint(Point &Point);
};

struct BSpline {
    int n;                  // 控制点数组下标
    int p;                  // 次数 n>=p, n=p时,为bezier曲线
    double *U;              // 节点数组, 长度(n+1)+(p+1)
    Point *Pw;              // 控制点数组, 长度n+1
};

enum {
    MAX_DEGREE = 23,    // same as libglu
    MIN_DEGREE = 1,
};

enum ERRCODE {
    SUCCESS = 0,        // 操作成功
    RECURSIVE_OVERFLOW, // 递归层次太深
    DEGREE_INVALID,     // 次数无效 小于MIN_DEGREE或者大于MAX_DEGREE
    MULTI_GREATER_THAN_ORDER,
    N_LESS_THAN_P,
    KNOTS_DECREASING,
    KNOTS_IS_UNCLAMPED, // 非端点插值的，或者周期的
    NOT_ENOUGH_MEMERY   // 内存不足
};

class BSplineSubdivision
{
public:
    BSplineSubdivision(int RecursionLimit = 32, double DefaultTolerance = 4.0, double MiniTolerance = 0.000001);

    void SetTolerance(double tolerance);
    double GetTolerance() { return mDistanceTolerance; }

    int DecomposeNurbsToLine(const BSpline &BSpline, PointArray &PtOut);
    int DecomposeCurve(int n, int p, const double *U, const Point *Pw, int &nb, Point *Qw);
private:
    int RecursiveBezier(Point Pts[], int Degree, int Level, PointArray &PtOut);
    int InterpolateBezier(int Degree, Point *ControlPts, PointArray &PtOut);
    int CheckKnotvector(const BSpline &BSpline);

    int CURVE_RECURSION_LIMIT;
    double MIN_TOLERANCE;

    double mDistanceTolerance;
};

}

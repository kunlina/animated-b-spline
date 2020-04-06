#ifndef GEOMETRYCOMPUTE_H
#define GEOMETRYCOMPUTE_H

#include <float.h>

struct Point {
    Point(): x(0.0), y(0.0), z(0.0), w(1.0) {}
    Point(double x, double y, double z, double w = 1.0): x(x), y(y), z(z), w(w) {}
    double x, y, z, w;
    inline Point &operator+=(const Point &p)
    {
        x+=p.x; y+=p.y; z+=p.z; return *this;
    }

    inline Point &operator-=(const Point &p)
    {
        x-=p.x; y-=p.y; z-=p.z; return *this;
    }

    inline Point &operator*=(double c)
    {
        x*=c; y*=c; z*=c; return *this;
    }

    inline Point &operator/=(double c)
    {
        x/=c; y/=c; z/=c; return *this;
    }

    friend inline const Point operator+(const Point &p)
    {
        return p;
    }

    friend inline const Point operator-(const Point &p)
    {
        return Point(-p.x, -p.y, -p.z);
    }

    friend inline const Point operator+(const Point &p1, const Point &p2)
    {
        return Point(p1.x+p2.x, p1.y+p2.y, p1.z+p2.z);
    }

    friend inline const Point operator-(const Point &p1, const Point &p2)
    {
        return Point(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z);
    }

    friend inline const Point operator*(const Point &p, double c)
    {
        return Point(p.x*c, p.y*c, p.z*c);
    }

    friend inline const Point operator*(double c, const Point &p)
    {
        return Point(p.x*c, p.y*c, p.z*c);
    }

    friend inline const Point operator/(const Point &p, double divisor)
    {
        return Point(p.x/divisor, p.y/divisor, p.z/divisor);
    }

    static inline double dotProduct(const Point &p1, const Point &p2)
    { return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z; }


    static inline bool isNull(double d)
    {
        if (d <= 0.000000000001 && d >= -0.000000000001) {
            return true;
        }
        return false;
    }

    static inline bool isNull(float d)
    {
        if (d <=  0.00001f && d >= -0.00001f) {
            return true;
        }
        return false;
    }

    void MidPoint(const Point &Pt1, const Point &Pt2);
    static double DistanceToLine(const Point &Pt, const Point &P1, const Point &P2);
};

/* 输出的小线段 */
struct PointArray {
    int validIndex;         // 已经使用的最后一个数的索引
    int totalNum;           // points空间的总长度
    Point *points;

public:
    int AddPoint(Point &Point);
};

/* 控制点 */
struct ControlPoints {
    int degree;             // 次数
    Point *points;          // 控制点数组
};

struct Nurbs {
    int n;                  // n + 1 = 控制点数
    int p;                  // 次数
    double *U;              // 节点数组
    Point *Pw;              // 控制点数组
};

class GeometryCompute
{
public:
    GeometryCompute();
    enum {
        MAX_DIMENSION = 4,
        MAX_DEGREE = 8,
        MIN_DEGREE = 2
    };

    enum ERRCODE {
        SUCCESS = 0,        // 操作成功
        DIM_INVALID,        // 维度无效
        PARA_INVALID,       // 参数无效
        RECURSIVE_OVERFLOW, // 递归层次太深
        DEGREE_INVALID,     // 次数无效
        DEGREE_TOO_HIGH,    // 次数大于 MAX_DEGREE
        DEGREE_TOO_LOW,     // 次数小于 MIN_DEGREE
        NOT_ENOUGH_MEMERY   // 内存不足
    };

    void SetTolerance(double tolerance);
    double GetTolerance() { return mDistanceTolerance; }
    double GetMinTolerance() { return MIN_TOLERANCE; }

    int DecomposeNurbsToLine(const Nurbs &Nurbs, PointArray &PtOut);

    int DecomposeCurve(int n, int p, const double *U, const Point *Pw, int &nb, Point *Qw);
    int InterpolateBezier(ControlPoints ControlPts, PointArray &PtOut);
    int InterpolateBezier(double BezierControlPts[], int Degree, int Dim,
                          double PtOut[], int Size, int *UsedSize);
private:
    int RecursiveBezier(Point Pts[], int Degree, int Level, PointArray &PtOut);

    int Recursive3DegreeBezier(double Pt1[], double Pt2[], double Pt3[], double Pt4[],
                               int Dim, int Level);

    int Recursive3DegreeBezier(Point Pt1, Point Pt2, Point Pt3, Point Pt4, int Level, PointArray &PtOut);

    static int DistanceToLine(const double Pt[], const double P1[], const double P2[], int Dim,
                              double *Distance);
    int AddPoint(double Pt[], int Dim);
    static void MidPoint(double P1[], double P2[], double PMid[], int Dim);
    int InterpolateBezier(int Degree, Point *ControlPts, PointArray &PtOut);

    int CURVE_RECURSION_LIMIT;
    double MIN_TOLERANCE;

    double mDistanceTolerance;
    double *mPtMem;
    int mPtIndex;
    int mPtTotalSize;
};

#endif // GEOMETRYCOMPUTE_H

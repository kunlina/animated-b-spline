#ifndef GEOMETRYCOMPUTE_H
#define GEOMETRYCOMPUTE_H

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

    static inline double dotProduct(const Point &p1, const Point &p2)
    { return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z + p1.w*p2.w; }


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

    static double DistanceToLine(const Point &Pt, const Point &P1, const Point &P2);
};

/* 输出的小线段 */
struct PointArray {
    int validIndex;         // 已经使用的个数
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
    int n;                  // n+1=控制点数
    int p;                  // 次数 n>=p, n=p时,为bezier曲线
    double *U;              // 节点数组, 长度(n+1)+(p+1)
    Point *Pw;              // 控制点数组, 长度n+1
    int nb;                 // bezier段数
    Point *Qw;              // 每个bezier段的控制点
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

private:
    int RecursiveBezier(Point Pts[], int Degree, int Level, PointArray &PtOut);
    int AddPoint(double Pt[], int Dim);
    int InterpolateBezier(int Degree, Point *ControlPts, PointArray &PtOut);

    int CURVE_RECURSION_LIMIT;
    double MIN_TOLERANCE;

    double mDistanceTolerance;
    double *mPtMem;
    int mPtIndex;
    int mPtTotalSize;
};

#endif // GEOMETRYCOMPUTE_H

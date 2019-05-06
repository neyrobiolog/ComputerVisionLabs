#ifndef CORNER_DETECTORS_H
#define CORNER_DETECTORS_H
#include "matrix.h"
#include "kernel.h"
#include "utils.h"
#include <cmath>
#include <limits>
#include <functional>
#include "kernel.h"
#include <vector>

using namespace std;
using namespace std::placeholders;

struct Point{
    int x;
    int y;
    double angle;
    double rad;
    double value;
    double sigma;
    Point(const int x,const int y,const double v):x(x),y(y),value(v){}
};

enum Algorithm {MORAVEC, HARIS};

class CornerDetectors
{
    const double E = 1e-5;
    const double sigma = 1.6;

    const double trasholdMor = 0.05;
    const double trasholdHar = 0.25;
    const int winSize = 3;
    const int pSize = 3;

public:
    CornerDetectors();
    vector<Point> detect(const Matrix & m,const Algorithm alg = MORAVEC) const;

    double computeHaris(const int i, const int j,const double sigma,
                                    const Matrix & derX, const Matrix & derY);
    // !!!
    double computeHaris(const Matrix & m, const int i, const int j, double sigma);

private:

    Matrix detectMoravec(const Matrix & m) const;
    Matrix detectHaris(const Matrix & m) const;

    vector<Point> localMinimums(const Matrix & m, const double tr) const;
    bool isMinimum(const Matrix & m,const int i,const int j, const double tr) const;
};

class PointFileter{
    const double factor = 0.9;
    int maxR;
    int maxPoints;

public:
    PointFileter(const int maxR, const int maxPoints);
    vector<Point> filter(const vector<Point> & points) const;
};


#endif // CORNER_DETECTORS_H

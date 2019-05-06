#ifndef SCALESPACE_H
#define SCALESPACE_H
#include "matrix.h"
#include <math.h>
#include <memory>
#include <vector>
#include <string>
#include "descriptor.h"

using namespace std;

struct ScaleLevel{
    Matrix matrix;
    double sigma;
    double efectSigma;
    ScaleLevel():sigma(0), efectSigma(0), matrix(Matrix(0, 0)){}
    ScaleLevel(ScaleLevel &&) = default;
    ScaleLevel(Matrix & m, double sigma, double efectSigma):
        sigma(sigma), efectSigma(efectSigma), matrix(move(m)){}
};

struct Blob{
    int layer;
    int octav;
    int x;
    int y;
    double sigma;
    Blob():layer(0),octav(0),x(0),y(0),sigma(0){}
    Blob(const Blob & other):
        layer(other.layer),octav(other.octav),x(other.x),y(other.y),sigma(other.sigma){}
    Blob(const int x, const int y, const int octav, const int layer, const double sigma):
        x(x),y(y),octav(octav),layer(layer),sigma(sigma){}
};

class ScaleSpace
{
private:
    const double EPS = 0.0001;
    const int _layerSize = 6;
    const double startSigma = 0.5;
    const double sigmaA = 1.6;
    const double k = pow(2,1/(double)_layerSize);

    vector<vector<ScaleLevel>> _octavs;

public:
    ScaleSpace(const int numOctavs);
    ScaleSpace(const Matrix &initMatrix,const int numOctavs);

    int octaveSize() const;
    int layerSize() const;
    double sigma0() const;

    const vector<vector<ScaleLevel>> & octavs() const;

    ScaleSpace computeDiffs() const;

    vector<Blob> searchBlobs() const;

private:
    bool checkExtremum(const int octav, const int level, const int i, const int j) const;
    Matrix gauss(const Matrix & matrix,const double sigma) const;
    void calculate(const Matrix & m);
};

class BlobFilter {
    const double treshold = 5.0;

public:
    vector<Blob> filter(const vector<Blob> & blobs, ScaleSpace & space) const;
};

class SIDiscrBuilder{

public:
   static vector<Descriptor> build(const Matrix & m);

};

class PointMatcher{

    const double eps;

public:
    PointMatcher(const double eps);
    vector<pair<Point,Point>> match(const Matrix &m1,
                                    const Matrix &m2, const bool withScale = false) const;
};

#endif // SCALESPACE_H

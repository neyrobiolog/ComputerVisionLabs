#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H
#include "corner_detectors.h"

using namespace std;

struct Descriptor{
    int x;
    int y;
    double angle;
    double rad;
    double sigma;
    vector<double> data;
    Descriptor(const int x, const int y, const double angle = 0);
    Descriptor(const int x, const int y,
               const vector<double> & data, const double angle = 0);
};

class DescrBuilder{
    const double treshold = 0.25;
    double sigma0;
    double sigma;

    const int maxNumPoints = 200;

    const int sizeHist = 4;
    const int numBins = 8;

    const int numDescrHist = 4;
    const int sizeDescrHist = 4;
    const int numDescrBins = 8;

    const int numRotateHist = 1;
    const int sizeRotateHist = 16;
    const int numRotateBins = 36;

    Matrix sobelX;
    Matrix sobelY;
    vector<Point> points;

public:
    DescrBuilder(const Matrix & m);
    DescrBuilder(const vector<Point> & points, Matrix & derX, Matrix & derY,
                                const double sigma, const double sigma0);
    vector<Descriptor> build() const;

private:
    vector<double> computeData(const Point &p,
            const double rotateAngle, const int numHist,
                               const int sizeHist, const int numBins) const;

    Descriptor filterTrash(const Descriptor &descr) const;
    Descriptor normilize(const Descriptor & descr) const;
    Descriptor normilizeAll(const Descriptor & descr) const;
};

// Поиск дескрипторов для двух изображений для четвертой лабы
class PointSearcher{

    const double eps = 0.05;

public:
    vector<pair<Point,Point>> findSamePoints( const Matrix &m1,const Matrix &m2) const;
};

#endif // DESCRIPTOR_H

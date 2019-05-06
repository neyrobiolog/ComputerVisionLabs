#ifndef HAUGH_H
#define HAUGH_H
#include <vector>
#include <memory>
#include <functional>
#include "descriptor.h"

using namespace std;

struct Progression
{
  double current;
  double multiplaer;
  double stap;

  Progression(double c, double k)
  {
      current = c, multiplaer=1/k, stap = k;
  }

  int operator()()
  {
      return current * (multiplaer*=stap);
  }
};

class Hough
{
    const int scaleBins = 6;
    const int angleBins = 6;
    const int coordBins = 20;
    const double scaleStap = 2;
    const double initScale = 1/10.0;

    vector<pair<Point, Point>> pairs;
    int centerX;
    int centerY;
    int numBinsX;
    int numBinsY;

public:
    // Структура параметров 4-х измерений
    struct HoughValue
    {
        double x;
        double y;
        double scale;
        double angle;

        HoughValue(double x,double y, double scale, double angle):
            x(x),y(y),scale(scale),angle(angle){}
    };

    Hough(const int width1, const int height1, const int width2, const int hegth2,
          const vector<pair<Point, Point>> pairs);

    HoughValue computeHaugh();

};

#endif // HAUGH_H

#include "corner_detectors.h"

CornerDetectors::CornerDetectors(){}

vector<Point> CornerDetectors::detect(const Matrix &m,
                                      const Algorithm alg) const
{
    Matrix matrix;
    double tr;
    switch (alg) {
    case MORAVEC:
        matrix = std::move(detectMoravec(m));
        tr = trasholdMor;
        break;
    case HARIS:
        matrix = std::move(detectHaris(m));
        tr = trasholdHar;
    default:
        break;
    }
    return localMinimums(matrix, tr);
}

// Детектор Харриса для шестой-седьмой лабы
double CornerDetectors::computeHaris(const int i, const int j,const double sigma,
                                     const Matrix & derX, const Matrix & derY)
{
    auto w = KernelFactory::createGauss(sigma);
    auto border = Matrix::Border::COPIED;
    int wSize = w.width/2;
    double A = 0, B = 0, C = 0;

    for (int v = 0; v < w.height; v++)
    {
        for (int u = 0; u < w.width; u++)
        {
            double Ix = derX.get(i + u - wSize, j + v - wSize, border);
            double Iy = derY.get(i + u - wSize, j + v - wSize, border);
            double k = w.matrix[u * w.height + v];

            A += k * Ix * Ix;
            B += k * Ix * Iy;
            C += k * Iy * Iy;
        }
    }

    double descr = std::sqrt(std::pow(A - C, 2) + 4 * B * B);
    return std::min(abs((A + C - descr)/2), abs((A + C + descr)/2));
}

// Детектор Харриса для шестой лабы (просто для построения блобов)
double CornerDetectors::computeHaris(const Matrix &m, const int i, const int j, double sigma)
{
    auto w = KernelFactory::createGauss(sigma);
    auto border = Matrix::Border::COPIED;
    auto kSobelX = KernelFactory::sobelX();
    auto kSobelY = KernelFactory::sobelY();
    int wSize = w.width/2;
    double A = 0, B = 0, C = 0;

    for (int v = 0; v < w.height; v++)
    {
        for (int u = 0; u < w.width; u++)
        {
            double Ix = m.convoluite(i + u - wSize, j + v - wSize, kSobelX, border);
            double Iy = m.convoluite(i + u - wSize, j + v - wSize, kSobelY, border);
            double k = w.matrix[u * w.height + v];

            A += k * Ix * Ix;
            B += k * Ix * Iy;
            C += k * Iy * Iy;
        }
    }
    double descr = std::sqrt(std::pow(A - C, 2) + 4 * B * B);
    return std::min(abs((A + C - descr)/2),abs((A + C + descr)/2));
}

Matrix CornerDetectors::detectMoravec(const Matrix &m) const
{
    auto errors = Matrix(m.width(),m.height());
    auto border = Matrix::Border::COPIED;
    for(int i = 0; i<m.width(); i++){
        for(int j = 0; j<m.height(); j++){
            double min = std::numeric_limits<double>::max();
            for(int u = -1; u <= 1; u++){
                for(int v = -1; v <= 1; v++){
                    if(!(u || v))
                        continue;
                    double err = 0;
                    for(int dx = -winSize; dx <= winSize; dx++){
                        for(int dy = -winSize; dy <= winSize; dy++){
                            err += pow(m.get(i+u+dx,j+v+dy,border) -
                                   m.get(i+dx,j+dy,border),2);
                        }
                    }
                    min = std::min(min, err);
                }
            }
            errors.set(i, j, min);
        }
    }
    return errors;
}

Matrix CornerDetectors::detectHaris(const Matrix &m) const
{
    auto lambdas = Matrix(m.width(),m.height());
    auto border = Matrix::Border::COPIED;
    auto derX = m.convolution(KernelFactory::sobelX(),border);
    auto derY = m.convolution(KernelFactory::sobelY(),border);
    auto w = KernelFactory::createGauss(sigma);
    int wSize = w.width/2;
    for(int i = 0; i < lambdas.width(); i++){
        for(int j = 0; j < lambdas.height(); j++){
            double A = 0, B = 0, C = 0;
            for(int v = 0; v < w.height; v++){
                for(int u = 0; u < w.width; u++){
                    double Ix = derX.get(i+u-wSize, j+v-wSize, border);
                    double Iy = derY.get(i+u-wSize, j+v-wSize, border);
                    double k = w.matrix[u*w.height+v];
                    A += k*Ix*Ix;
                    B += k*Ix*Iy;
                    C += k*Iy*Iy;
                }
            }
            double descr = std::sqrt(std::pow(A-C,2) + 4*B*B);
            lambdas.set(i, j, std::min(abs((A+C-descr)/2),abs((A+C+descr)/2)));
        }
    }
    return lambdas;
}

vector<Point> CornerDetectors::localMinimums(const Matrix &m, const double tr) const
{
    vector<Point> points;
    for(int i = 0; i < m.width(); i++){
        for(int j = 0; j < m.height(); j++){
            if(isMinimum(m, i, j, tr))
                points.emplace_back(move(Point(i, j, m.get(i,j))));
        }
    }
    return points;
}

bool CornerDetectors::isMinimum(const Matrix &m, const int i,
                                const int j, const double tr) const
{
    const auto value = m.get(i, j);
    if(value < tr)
        return false;
    for(int k = -pSize; k < pSize; k++){
        for(int t = -pSize; t < pSize; t++){
            if(m.get(i+k,j+t,Matrix::Border::SIMPLE) - value > E){
                return false;
            }
        }
    }
    return true;
}

PointFileter::PointFileter(const int maxR, const int maxPoints):
    maxR(maxR),maxPoints(maxPoints){}

vector<Point> PointFileter::filter(const vector<Point> &points) const
{
    vector<Point> result(points);
    for(int rad = 0; rad < maxR && result.size() > maxPoints; rad++){
        for(int i = 0; i < result.size(); i++){
            for(int j = i+1; j < result.size(); j++){
                if(sqrt(pow(result[i].x - result[j].x,2) +
                       pow(result[i].y - result[j].y,2)) <= rad &&
                        factor*result[i].value > result[j].value){
                    result.erase(result.begin() + j);
                    break;
                }
            }
        }
    }
    return result;
}

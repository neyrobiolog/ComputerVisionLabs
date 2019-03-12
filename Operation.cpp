#include <iostream>
#include <memory>
#include "Operation.h"
#include "Kernels.h"

using namespace  std;

Matrix<int> Operation::g_sobelX(3, 3, VsobelX);
Matrix<int> Operation::g_sobelY(3, 3, VsobelY);

Matrix<int> Operation::g_prewittX(3, 3, VprewittX);
Matrix<int> Operation::g_prewittY(3, 3, VprewittY);

Matrix<int> Operation::g_scharrX(3, 3, VscharrX);
Matrix<int> Operation::g_scharrY(3, 3, VscharrY);

const vector<pair<int,int>> Operation::g_shiftWindow =
              { {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1} , {-1,-1} };

// Конструктор
Operation::Operation() {}

// Перевод изображения в оттенки серого
void Operation::grayScale(QImage& image, Image& myImage)
{
    for (int i = 0; i < image.height(); i++)
    {
        QRgb *pixel = reinterpret_cast<QRgb*>(image.scanLine(i));
        QRgb *end = pixel + image.width();

        for (auto j = 0; pixel != end; pixel++, j++)
        {
            QColor clrCurrent(*pixel);
            int grayPixel = gray(clrCurrent.red(),
                                 clrCurrent.green(),
                                 clrCurrent.blue());
            myImage.setItem(i, j, grayPixel);
        }
    }
}

// Формула получения оттенка серого для одного пикселя
int Operation::gray(byte r, byte g, byte b)
{
    return (0.299 * r + 0.587 * g + 0.114 * b);
}

//*********************************
//         ОПЕРАТОР СОБЕЛЯ
//*********************************

// Оператор Собеля оригинальный
void Operation::sobel(EDGEWORKTYPE method, Image& image)
{
    magnitude(image, convolution(g_sobelX, image, method), convolution(g_sobelY, image, method));
}

// Оператор Собеля: производная по X
void Operation::sobelX(EDGEWORKTYPE method, Image& image)
{
    magnitude(image, convolution(g_sobelX, image, method), convolution(g_sobelX, image, method));
}

// Оператор Собеля: производная по Y
void Operation::sobelY(EDGEWORKTYPE method, Image& image)
{
    magnitude(image, convolution(g_sobelY, image, method), convolution(g_sobelY, image, method));
}

// Оператор Собеля по Прюитту
void Operation::priwitt(EDGEWORKTYPE method, Image& image)
{
    magnitude(image, convolution(g_prewittX, image, method), convolution(g_prewittY, image, method));
}

// Оператор Собеля по Щарру
void Operation::scharr(EDGEWORKTYPE method, Image& image)
{
    magnitude(image, convolution(g_scharrX, image, method), convolution(g_scharrY, image, method));
}

// Вычисление величины градиента
void Operation::magnitude(Image& input, const Image& gx, const Image& gy)
{
    for (auto y = 0; y < input.getHeight(); y++)
    {
        for (auto x = 0; x < input.getWidth(); x++)
        {
            input.setItem(y, x, sqrt(gx.getItem(y, x) * gx.getItem(y, x) +
                                      gy.getItem(y, x) * gy.getItem(y, x)));
        }
    }
}

//*********************************
//          ФИЛЬТР ГАУССА
//*********************************

// Фильтр Гаусса
void Operation::gaussianBlur(float sigma, Image& myImage, EDGEWORKTYPE method)
{
    convolutionForGauss(sigma, myImage, method);
}

// Свертка для Гаусса
void Operation::convolutionForGauss(float sigma, Image& myImage, EDGEWORKTYPE method)
{
    // Свертка по свойству сепарабельности:
    // сперва по ширине, потом по высоте изображения
    vector<float> temp = gaussianKernel(sigma);
    const Matrix<float> Gaus1W(1, temp.size(), temp);
    const Matrix<float> Gaus1H(temp.size(), 1, temp);

    auto g1 = convolution(Gaus1W, myImage, method);
    auto g2 = convolution(Gaus1H, g1, method );

    myImage = std::move(g2);
}

// Вычисление одностороннего ядра фильтра Гаусса
vector<float> Operation::gaussianKernel(float sigma)
{
    // Правило "трех сигм" для размера ядра
    unsigned sizeKernel = 3 * sigma * 2;

    // Проверка на достаточный размер и нечётность ядра
    if (sizeKernel < 2)
        sizeKernel = 3;
    if (sizeKernel % 2 == 0)
        sizeKernel++;

    vector<float> myGaussKernel;
    myGaussKernel.resize(sizeKernel, 0);
    int edgeKernel = sizeKernel/2;
    float summ = 0;

    // Заполняем матрицу ядра Гаусса коэффициентами
    for (int i = 0, x = -edgeKernel; x <= edgeKernel; x++, i++)
    {
        auto cur = gaussian(x, sigma);
        myGaussKernel[i]= cur;
        summ += cur;
    }

    // Нормализация ядра
    for (size_t i = 0; i < myGaussKernel.size(); i++)
    {
        myGaussKernel[i] /= summ;
    }

    return myGaussKernel;
}

// Формула фильтра Гаусса по свойству сепарабельности
float Operation::gaussian(int x, float sigma)
{
    return  exp( -(x * x) / (2 * sigma * sigma)) / (sqrt(2 * PI) * sigma);
}

//*********************************
//         ПИРАМИДА ГАУССА
//*********************************

// Формула Пифагора
float pythagoras(float sigmaNext, float sigmaPrev)
{
    return sqrt(sigmaNext * sigmaNext - sigmaPrev * sigmaPrev);
}

// Формирование Пирамиды Гаусса
Pyramid Operation::gaussPyramid(Image& img, int octaves, int scales, float sigmaZero, EDGEWORKTYPE method)
{
    float sigmaPrev;
    float sigmaNext;
    float deltaSigma;
    sigmaPrev = 0.5;                                // Изначальное значение сигмы
    float k = pow(2, (float)1 / scales);           // Интервал между масштабами в октаве
    deltaSigma = pythagoras(sigmaZero, sigmaPrev);
    Image temp = img;

    // Сглаживаем оригинальное изображение, кладем его как нулевую октаву
    gaussianBlur(sigmaPrev, temp, method);
    Pyramid pyramid(octaves + 1, sigmaZero, scales);
    pyramid.setImageInOctaves(temp, 0, sigmaPrev);

    sigmaNext = sigmaZero;

    // Вычисляем для остальных октав
    for (int i = 1; i <= octaves; i++)
    {
        // Блюрим по масштабам
        for (int j = 0; j < scales; j++)
        {
             sigmaPrev = sigmaNext;
             sigmaNext = sigmaPrev * k;
             deltaSigma = pythagoras(sigmaNext, sigmaPrev);
             gaussianBlur(deltaSigma, temp, method);

             // Сохраняем каждый масштаб в данной октаве
             //    QString fileName = QString("octave_%1_scales_%2_sigma_%3.png").arg(i).arg(j).arg(sigmaPrev);
             //    temp.getImage().save(fileName);
        }
        downSpace(temp);
        pyramid.setImageInOctaves(temp, i, deltaSigma);

        QString fileName = QString("octave_%1_ssDownSpace_sigma_%2.png").arg(i).arg(sigmaPrev);
        temp.getImage().save(fileName);
    }
    return pyramid;
}

// Уменьшение размера изображения в два раза
void Operation::downSpace(Image& myImg)
{
    int newW = myImg.getWidth()/2;
    int newH = myImg.getHeight()/2;
    vector<float> temp = resizeBilinear(myImg, myImg.getWidth(), myImg.getHeight(), newW, newH);
    myImg.resize(newH, newW, temp);
    temp.clear();
}

// Собственно уменьшение размера изображения методом биленейной интерполяции
vector<float> Operation::resizeBilinear(const Image& img, int widthOld, int heightOld, int widthNew, int heightNew)
{
    vector<float> temp;
    temp.resize(widthNew * heightNew);
    int a, b, c, d, x, y, index;
    float x_ratio = ((float)(widthOld - 1)) / widthNew;
    float y_ratio = ((float)(heightOld - 1)) / heightNew;
    float x_diff, y_diff;
    int offset = 0;

    for (auto i = 0; i < heightNew; i++)
    {
        for (auto j = 0; j < widthNew; j++ )
        {
            x = (int)(x_ratio * j);
            y = (int)(y_ratio * i);
            x_diff = (x_ratio * j) - x;
            y_diff = (y_ratio * i) - y;
            index = (y * widthOld + x);

            a = img.getItem(0, index);
            b = img.getItem(0, index + 1 );
            c = img.getItem(0, index + widthOld);
            d = img.getItem(0, index + widthOld + 1);

            temp[offset++] =  a * (1 - x_diff) * (1 - y_diff) +
                              b * (x_diff) * (1 - y_diff) +
                              c * (y_diff)* (1 - x_diff) +
                              d * (x_diff * y_diff);
        }
    }

    return temp;
}

//*********************************
//  ДЕТЕКТОРЫ ХАРРИСА И МОРАВИКА
//*********************************

// Детектор Моравика
vector<QPoint> Operation::moravec(const Image& myImage, float T, size_t windowHeight, size_t windowWidth,
                                  bool useNonMaximum, int colPoints)
{
    auto offsetx = windowWidth / 2;
    auto offsety = windowHeight / 2;
    vector<QPoint> point;
    vector<float> values;

    int p = 3;
    auto offestp = p / 2;

    for (size_t y = 1 + offsety + offestp; y < myImage.getHeight() - offsety - offestp - 1; y++)
    {
        for (size_t x = 1 + offsetx + offestp; x < myImage.getWidth() - offsetx - offestp  - 1; x++)
        {
            // Ошибка при сдвигах окна
            float minError = minErrorShift(x, y, windowHeight, windowWidth, myImage);

            if (filtrate(x, y, minError, T, myImage, p, windowHeight, windowWidth, myImage, myImage, 0))
            {
                point.push_back(QPoint(x, y));
                values.push_back(minError);
            }

        }
    }

    // Алгоритм подавления немаксимумов
    if (useNonMaximum)
        nonMaximumPoints(values, point, colPoints);

    return point;
}

// Ошибка при сдвигах окна для Детектора Моравика
float Operation::minErrorShift(int x, int y, size_t windowHeight, size_t windowWidth, const Image& myImage)
{
    vector<float> ErrorShift;
    ErrorShift.resize(g_shiftWindow.size());

    for (size_t sh = 0; sh < g_shiftWindow.size(); sh++ )
    {
        ErrorShift[sh] = valueErrorShift(x, y, sh, windowHeight, windowWidth, myImage);
    }

    auto minErrorShift = std::min_element(ErrorShift.begin(), ErrorShift.end());

    return *minErrorShift;
}

// Подсчет ошибки при сдвигах окна для Детектора Моравика
float Operation::valueErrorShift(int x, int y, int sh, size_t windowHeight, size_t windowWidth, const Image& myImage)
{
    auto offsetx = windowWidth / 2;
    auto offsety = windowHeight / 2;
    float sum = 0;

    // C(x, y, d) = Eu Ev [I (x + u, y + v) - I (x + u + dx, y + v + dy)]^2
    for (size_t j = 0; j < windowHeight; j++)
    {
        for (size_t i = 0; i < windowWidth; i++)
        {
            float dif = myImage.getItem(y + j - offsety, x + i - offsetx) -
                        myImage.getItem(y + j - offsety + g_shiftWindow[sh].first,
                                         x + i - offsetx + g_shiftWindow[sh].second);
            dif = dif * dif;
            sum += dif;
        }
    }

    return sum;
}

// Требование локального максимума
bool Operation::filtrate(int x, int y, float valueOperator, float T, const Image& myImage, int ambit,
                             int windowHeight, int windowWidth, const Image& dx, const Image& dy, float k)
{
    // Требование порогового значения
    if(valueOperator < T)
        return false;

    // Окрестность
    auto offestAmbit = ambit / 2;

    // Требование локального максимума
    for (int py = 0 ; py < ambit; py++)
    {
        for (int px = 0; px < ambit; px++)
        {
            float valueOperatorInAmbit = 0;

            if (windowHeight != 0 && windowWidth != 0)
            {
                valueOperatorInAmbit = minErrorShift(x + px - offestAmbit,
                                                     y + py - offestAmbit,
                                                     windowHeight, windowWidth, myImage );
            }

            else if (k != 0)
            {
                valueOperatorInAmbit = eigenvaluesHarris(x + px - offestAmbit,
                                                         y + py - offestAmbit,
                                                         dx, dy, k, ambit);
            }

            if (valueOperator < valueOperatorInAmbit)
                return false;
        }
    }

    return true;
}

// Детектор Харриса
vector<QPoint> Operation::harris(const Image& myImage, float T , float k, bool useNonMaximum, int colPoints)
{
    // Оператор Собеля
    auto dx = convolution(g_sobelX, myImage, COPYEDGE);
    auto dy = convolution(g_sobelY, myImage, COPYEDGE);

    vector<QPoint> point;

    vector<float> value;
    size_t p = 3;
    auto offsetp = p/2;

    for (auto y = 1 + offsetp ; y < myImage.getHeight() - 1 - offsetp; y++)
    {
        for (auto x = 1 + offsetp; x < myImage.getWidth() - 1 - offsetp; x++)
        {
            float M = eigenvaluesHarris(x, y, dx, dy, k, p);

            if (filtrate(x, y, M, T, myImage, p , 0 , 0, dx, dy, k))
            {
                value.push_back(M);
                point.push_back(QPoint(x, y));
            }
        }
    }

    // Алгоритм подавления немаксимумов
    if (useNonMaximum)
        nonMaximumPoints(value, point, colPoints);

    return point;
}

// Ошибка при сдвигах окна для Детектора Харриса (собственные числа Детектора Харриса)
float Operation::eigenvaluesHarris(const int x, const int y, const Image& dx,
                                       const Image& dy, const float k, const int ambit)
{
    float A = 0;
    float B = 0;
    float C = 0;

    // Окрестность
    auto offestp = ambit / 2;

    for (int py = 0; py < ambit; py++)
    {
        for (int px = 0; px < ambit; px++)
        {
           float dxv = dx.getItem(y + py - offestp, x + px - offestp);
           float dyv = dy.getItem(y + py - offestp, x + px - offestp);
           A += dxv * dxv;
           B += dxv * dyv;
           C += dyv * dyv;
        }
    }

    return (A * C - B * B) - k *((A + C) * ( A + C ));
}

// Расстояние между точками
float Operation::distanceBetweenPoints(const QPoint& p1, const QPoint& p2)
{
    return sqrt(((p1.x() - p2.x()) * (p1.x() - p2.x())) + ((p1.y() - p2.y()) * (p1.y() - p2.y())));
}

// Алгоритм подавления немаксимумов
vector<QPoint> Operation::nonMaximumPoints(vector<float>& value, vector<QPoint>& points, int colPoints)
{
    // Радиус окрестности
    int r = 3;

    while (points.size() > colPoints)
    {
        for (size_t i = 0 ; i < points.size(); i++)
        {
            for (size_t j = i + 1; j < points.size(); j++)
            {
                if (distanceBetweenPoints(points[i], points[j]) < r)
                {
                    if (value[i] < value[j])
                    {
                        value.erase(value.begin() + i);
                        points.erase(points.begin() + i);
                        i--;
                    }
                }
            }
        }
        r++;
    }

    return points;
}

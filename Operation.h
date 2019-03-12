#ifndef OPERATION_H
#define OPERATION_H
#include "Image.h"
#include "Pyramid.h"
#include <QDebug>
#include <QPoint>
#include <cmath>

// Обработка краев изображения
enum EDGEWORKTYPE
{
    COPYEDGE,  // Копировать, сколько требуется, значение с края изображения
    BLACKEDGE, // Считать, что снаружи 0 (все черное)
    WRAPEDGE   // "Заворачивать" изображение у края
};

class Operation
{
private:

    typedef unsigned char byte;
    float PI = 3.14;

    static Matrix<int> g_sobelX;          // Ядро Собеля по X
    static Matrix<int> g_sobelY;          // Ядро Собеля по Y
    static Matrix<int> g_prewittX;        // Ядро Привитта по X
    static Matrix<int> g_prewittY;        // Ядро Привитта по Y
    static Matrix<int> g_scharrX;         // Ядро Щарра по X
    static Matrix<int> g_scharrY;         // Ядро Щарра по Y
    static const std::vector<std::pair<int,int>> g_shiftWindow;     // Окно Детектора Моравика

public:

    Operation();

    // Оператор Собеля
    void sobel(EDGEWORKTYPE method, Image& image);
    void sobelX(EDGEWORKTYPE method, Image& image);
    void sobelY(EDGEWORKTYPE method, Image& image);

    // Функция перевода изображения в оттенки серого
    void grayScale(QImage& image, Image& myImage);

    // Фильтр Гаусса
    void gaussianBlur(float sigma, Image& myImage, EDGEWORKTYPE method = BLACKEDGE);

    // Оператор Робертса
    void scharr(EDGEWORKTYPE method, Image& image );

    // Оператор Привитта
    void priwitt(EDGEWORKTYPE method, Image& image );

    // Пирамида Гаусса
    Pyramid gaussPyramid(Image& img, int octaves, int sclaes, float sigmaZero, EDGEWORKTYPE method);

    // Детектор Харриса
    std::vector<QPoint> harris(const Image& myImage, float T , float k, bool useNonMaximum, int colPonts);

    // Детектор Моравика
    std::vector<QPoint> moravec(const Image& myImage, float T, size_t windowHeight, size_t windowWidth, bool useNonMaximum, int colPoints);

private:

    // Формула получения оттенка серого для одного пикселя
    int gray(byte r, byte g, byte b);

    // Вычисление величины градиента
    void magnitude(Image& input, const Image& gx, const Image& gy);

    // Свертка для Гаусса по свойству сепарабельности
    void convolutionForGauss(float sigma, Image& myImage, EDGEWORKTYPE method);

    // Вычисление одностороннего ядра фильтра Гаусса
    std::vector<float> gaussianKernel(float sigma);

    // Формула фильтра Гаусса по свойству сепарабельности
    float gaussian(int x, float s);

    // Собственно свёртка
    template<typename T>
    Image convolution(const Matrix<T>& kernel, const Image& myImage, EDGEWORKTYPE method)
    {
        int kernelWigth = kernel.getRows();
        int kernelHeight = kernel.getColumns();
        auto pixelPosX = kernelWigth/2;
        auto pixelPosY = kernelHeight/2;
        auto widthImage = myImage.getWidth();
        auto heightImage = myImage.getHeight();
        float summ;

        // Результирующее изображение
        Image outImage(heightImage, widthImage);

        for (auto y = 0; y < heightImage; y++)
        {
            for (auto x = 0; x < widthImage; x++)
            {
                summ = 0;
                bool flag = false;

                for (auto j = 0; j < kernelHeight; j++)
                {
                    if ((y + j < pixelPosY || y + j >= heightImage))
                    {
                        if (method == BLACKEDGE)
                            continue;
                        flag = true;
                    }

                    for (auto i = 0; i < kernelWigth; i++)
                    {
                        if (x + i < pixelPosX || x + i >= widthImage)
                        {
                            if (method == BLACKEDGE)
                                continue;

                            if (method == COPYEDGE)
                            {
                                summ += kernel.getItem(j, i) * myImage.getItem(y, x);
                                continue;
                            }
                        }

                        if (method == WRAPEDGE)
                        {
                            if (y + j < pixelPosY)
                            {
                                if (x + i < pixelPosX)
                                {
                                    summ += kernel.getItem(j, i) * myImage.getItem(heightImage, widthImage);
                                }
                                else if (x + i >= widthImage)
                                {
                                    summ += kernel.getItem(j, i) * myImage.getItem(heightImage - 1 , widthImage);
                                }
                                else
                                    summ += kernel.getItem(j, i) * myImage.getItem(y + j - pixelPosY, x + i - pixelPosX);
                                continue;
                            }

                            if (y + j >= heightImage)
                            {
                                if (x + i < pixelPosX)
                                {
                                    summ += kernel.getItem(j, i) * myImage.getItem(0, widthImage) ;
                                }
                                else if (x + i >= widthImage)
                                {
                                    summ += kernel.getItem(j, i) * myImage.getItem(0, 0);
                                }
                                else
                                    summ += kernel.getItem(j, i) * myImage.getItem(y + j - pixelPosY, x + i - pixelPosX);
                                continue;
                            }
                        }

                        else if (flag)
                        {
                            if (method == COPYEDGE)
                            {
                                summ += kernel.getItem(j, i) * myImage.getItem(y, x + i - pixelPosX);
                                continue;
                            }

                            if (method == WRAPEDGE)
                            {
                                if (y + j < pixelPosY)
                                {
                                    summ += kernel.getItem(j, i) * myImage.getItem(heightImage, x + i - pixelPosX);
                                    continue;
                                }

                                if (y + j >= heightImage)
                                {
                                    summ += kernel.getItem(j, i) * myImage.getItem(0, x + i - pixelPosX);
                                    continue;
                                }
                            }
                        }

                        summ += kernel.getItem(j, i) * myImage.getItem(y + j - pixelPosY, x + i - pixelPosX);
                    }
                }
                outImage.setItem(y, x, summ);
            }
        }

        // Нормализация
        // outImage.normalizeImage();

        return outImage;
    }

    // Сжатие изображения
    std::vector<float> resizeBilinear(const Image& myImage, int widthOld, int heightOld, int widthNew, int heightNew);

    // Уменьшение изображения в 2 раза
    void downSpace(Image& myImage);

    // Отметить на ихображении точки интереса
    QImage setRedPointsOfInterest(Image& myImage, std::vector<QPoint> interestPoints);

    // Вычисляем значение ошибки при сдвиге
    float valueErrorShift(int x, int y, int sh, size_t windowHeight, size_t windowWidth, const Image& myImage);

    // Фильтруем точки интереса
    bool filtrate(int x, int y, float valueOperator, float T, const Image& myImage,int ambit, int windowHeight, int windowWidth, const Image& dx, const Image& dy, float k);

    // Подавление не максимальных элементов
    std::vector<QPoint> nonMaximumPoints(std::vector<float>& value, std::vector<QPoint>& points, const int colPoints);

    // Растояние между 2 точка
    float distanceBetweenPoints( const QPoint& p1, const QPoint& p2);

    // Собстевенные числа для Харрисона
    float eigenvaluesHarris(int x, int y, const Image& dx, const Image& dy, float k, int ambit);

    // Минимальная ошибка при сдвигах в 8 направлениях
    float minErrorShift(int x, int y, size_t windowHeight, size_t windowWidth, const Image& myImage);
};

#endif // OPERATION_H

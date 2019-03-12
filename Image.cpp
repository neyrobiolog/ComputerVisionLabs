#include "Image.h"

using namespace  std;

// Конструктор по умолчанию
Image::Image() {}

// Конструктор с параметрами
Image::Image(int height, int width)
{
    Matrix<float> myImage(height, width);
    m_myImage = std::move(myImage);
}

// Конструктор с параметрами
Image::Image(int height, int width, vector<float>& img)
{
    Matrix<float> myImage(height, width, img);
    m_myImage = std::move(myImage);
}

// Конструктор копирования
Image::Image (const Image & image)
{
    m_myImage = image.m_myImage;
}

// Конструктор перемещения
Image::Image (Image&& image)
{
   m_myImage = std::move(image.m_myImage);
}

// Оператор присваивания
Image& Image::operator= (const Image& image)
{
    m_myImage = image.m_myImage;
    return *this;
}

// Оператор перемещения
Image&  Image::operator= (Image&& image)
{
    if (this != &image)
    {
        m_myImage.getMatrix().clear();
        m_myImage = std::move(image.m_myImage);
    }
    return *this;
}

// Деструктор
Image::~Image(){}

// Получить изображение по его матрице яркости
QImage Image::getImage()
{
    QImage img (m_myImage.getRows(), m_myImage.getColumns(), QImage::Format_RGB32);

    m_myImage.normalize();

    for (int i = 0; i < m_myImage.getColumns(); i++)
    {
        QRgb *pixel = reinterpret_cast<QRgb*>(img.scanLine(i));
        QRgb *end = pixel + m_myImage.getColumns();

        for (int j =0; pixel != end; pixel++, j++)
        {
            int item = m_myImage.getItem(i, j);
            *pixel = QColor(item, item, item).rgb();
        }
    }

    return img;
}

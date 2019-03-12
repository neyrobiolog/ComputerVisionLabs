#ifndef IMAGE_H
#define IMAGE_H
#include <vector>
#include <QImage>
#include "QPixmap"
#include "Matrix.h"

class Image
{
private:

    Matrix<float> m_myImage;                  // Матрица яркости изображения

public:

    Image ();                                  // Конструктор по умолчанию
    Image (int height, int width );          // Конструктор с параметрами
    Image (const Image & image);              // Конструктор копирования
    Image (Image&& image);                    // Конструктор перемещения
    Image& operator= (const Image& image);    // Оператор присваивания
    Image& operator= (Image&& image);         // Оператор перемещения
    ~Image();                                  // Деструктор
    Image(int _height, int _width, std::vector<float>& img); // Конструктор
    QImage getImage();                         // Получить изображение по его матрице яркости

    // Высота изображения
    int getHeight() const
    {
        return m_myImage.getColumns();
    }

    void setHeight(int value)
    {
        m_myImage.setColumns(value);
    }

    // Ширина изображения
    int getWidth() const
    {
        return m_myImage.getRows();
    }

    void setWidth(int value)
    {
        m_myImage.setRows(value);
    }

    // Изенение размера
    void resize(int columns, int rows, std::vector<float>& newImg)
    {
        m_myImage.resize(columns, rows, newImg);
    }

    // Нормализация
    void normalizeImage()
    {
        m_myImage.normalize();
    }

    // Пиксели изображения
    void setItem(int columns, int rows, float value)
    {
        m_myImage.setItem(columns, rows, value);
    }

    // Получить изображение из его матрицы яркости
    float getItem(int columns, int rows) const
    {
        return m_myImage.getItem(columns, rows);
    }
};

#endif // IMAGE_H
